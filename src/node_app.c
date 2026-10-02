/* node_app.c
 * Core Relay & Control — node state machine.
 * Owner: Prabhav K
 *
 * ---------------------------------------------------------------------
 * INTERRUPT PRIORITY TABLE (set in main.c / NVIC config, documented here
 * so the reasoning lives next to the code that depends on it):
 *
 *   Priority 0 (highest): Relay UART RX  -- a lost byte here breaks the
 *                          whole relay chain; nothing else is as costly
 *                          to get wrong.
 *   Priority 1:            Sampling timer (Rohan's 1 kHz ADC timer) --
 *                          must stay on schedule or detection timing
 *                          and the filter window drift.
 *   Priority 2:            Beacon-blink timer (Praneeth) -- a dropped
 *                          tick just delays a blink edge by <1 ms;
 *                          visible but not safety-critical.
 *   Priority 3:            SysTick -- used for timestamps and the 5 ms
 *                          RX timeout check in Comm_Poll(); coarse
 *                          enough to tolerate some jitter.
 *   Priority 4 (lowest):   Log UART TX (Roshan) -- purely informational,
 *                          never allowed to delay anything above it.
 *
 * ---------------------------------------------------------------------
 * LATENCY BUDGET (fill in the measured column from Gunadeep's logic
 * analyzer captures once available; targets below are design intent):
 *
 *   Stage                                   Target     Measured
 *   ADC sample -> Sensing sets hazard flag   <= 1 ms    ____
 *   Node_Run() notices flag (worst case,
 *     one main-loop iteration)               <= 0.2 ms  ____
 *   Comm_SendHazard() -> byte on the wire    <= 0.1 ms  ____
 *   6 bytes at 115200 baud (10 bits/byte)     ~0.52 ms  ____
 *   Comm_Poll() dispatch -> forward decided   <= 0.2 ms  ____
 *   -----------------------------------------------------
 *   Per hop, end to end                      ~2 ms      ____
 *   Node 1 -> Node 3 (2 hops)                 ~4 ms      ____   (budget: 10 ms)
 * ---------------------------------------------------------------------
 */
#include "node_app.h"
#include "comm.h"
#include "config.h"      /* NODE_ID, IS_LAST_NODE, BEACON_MS            */
#include "sensing.h"      /* Sensing_HazardDetected/GetSeverity/ClearHazard - Rohan */
#include "alert.h"        /* Alert_BeaconOn/BeaconOff                    - Praneeth */
#include "log.h"           /* Log_Event                                   - Roshan   */
#include "stm32f4xx_hal.h"

/* ------------------------------------------------------------------ */
/* State machine                                                       */
/* ------------------------------------------------------------------ */
typedef enum {
    STATE_IDLE = 0,
    STATE_LOCAL_HAZARD,
    STATE_RELAYED_HAZARD,
    STATE_COOLDOWN
} NodeState;

static volatile NodeState s_state;
static volatile uint32_t  s_beaconStartTick;
static uint8_t s_nextSeq;   /* this node's own sequence counter, for LOCAL hazards */

/* ------------------------------------------------------------------ */
/* Duplicate suppression: remember the last few (SRC, SEQ) pairs seen */
/* ------------------------------------------------------------------ */
#define SEEN_TABLE_LEN  8u

typedef struct {
    uint8_t src;
    uint8_t seq;
    uint8_t valid;
} SeenEntry;

static SeenEntry s_seen[SEEN_TABLE_LEN];
static uint8_t   s_seenNext;   /* circular insertion index */

static uint8_t AlreadySeen(uint8_t src, uint8_t seq)
{
    for (uint8_t i = 0; i < SEEN_TABLE_LEN; i++) {
        if (s_seen[i].valid && s_seen[i].src == src && s_seen[i].seq == seq) {
            return 1;
        }
    }
    return 0;
}

static void RememberSeen(uint8_t src, uint8_t seq)
{
    s_seen[s_seenNext].src = src;
    s_seen[s_seenNext].seq = seq;
    s_seen[s_seenNext].valid = 1;
    s_seenNext = (uint8_t)((s_seenNext + 1) % SEEN_TABLE_LEN);
}

/* ------------------------------------------------------------------ */
/* Internal helpers                                                     */
/* ------------------------------------------------------------------ */
static void EnterCooldown(void)
{
    s_state = STATE_COOLDOWN;
    s_beaconStartTick = HAL_GetTick();
}

static void HandleLocalHazard(void)
{
    uint8_t sev = Sensing_GetSeverity();
    uint8_t seq = s_nextSeq++;

    Log_Event("LOCAL_HAZARD", NODE_ID, seq, sev);
    Alert_BeaconOn(sev);
    Comm_SendHazard(NODE_ID, seq, sev, /*hops=*/0);
    Log_Event("RELAY_TX", NODE_ID, seq, sev);

    RememberSeen(NODE_ID, seq);   /* so we don't re-act if it ever loops back */
    s_state = STATE_LOCAL_HAZARD;
    EnterCooldown();
}

static void HandleCooldownExpiry(void)
{
    if (HAL_GetTick() - s_beaconStartTick >= BEACON_MS) {
        Alert_BeaconOff();
        Log_Event("BEACON_OFF", NODE_ID, 0, 0);
        Sensing_ClearHazard();
        s_state = STATE_IDLE;
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */
void Node_App_Init(void)
{
    s_state = STATE_IDLE;
    s_nextSeq = 0;
    s_seenNext = 0;
    for (uint8_t i = 0; i < SEEN_TABLE_LEN; i++) {
        s_seen[i].valid = 0;
    }
}

void Node_Run(void)
{
    /* Let the comm module dispatch any fully-received packet. If one is
     * ready, Comm_OnHazardPacket() below runs synchronously inside this
     * call, before Comm_Poll() returns. */
    Comm_Poll();

    switch (s_state) {
        case STATE_IDLE:
            if (Sensing_HazardDetected()) {
                HandleLocalHazard();
            }
            break;

        case STATE_LOCAL_HAZARD:
        case STATE_RELAYED_HAZARD:
            /* Nothing to do here but wait for the cooldown to be entered
             * (it already was, inside HandleLocalHazard / the packet
             * callback) — kept as explicit states for the report's state
             * diagram and for future extension (e.g. re-triggering). */
            break;

        case STATE_COOLDOWN:
            HandleCooldownExpiry();
            break;

        default:
            s_state = STATE_IDLE;
            break;
    }
}

/* ------------------------------------------------------------------ */
/* Called by comm.c from Comm_Poll() (main-loop context, not the ISR)  */
/* ------------------------------------------------------------------ */
void Comm_OnHazardPacket(uint8_t src, uint8_t seq, uint8_t sev, uint8_t hops)
{
    if (AlreadySeen(src, seq)) {
        /* Duplicate: a retransmit, a loop, or a replay in a test.
         * Scenario S5 checks exactly this path. */
        return;
    }
    RememberSeen(src, seq);

    Log_Event("RELAY_RX", src, seq, sev);
    Alert_BeaconOn(sev);

    if (!IS_LAST_NODE) {
        Comm_SendHazard(src, seq, sev, (uint8_t)(hops + 1));
        Log_Event("RELAY_TX", src, seq, sev);
    }

    s_state = STATE_RELAYED_HAZARD;
    EnterCooldown();
}
