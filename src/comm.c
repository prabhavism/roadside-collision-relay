/* comm.c
 * Core Relay & Control — node-to-node hazard relay over UART.
 * Owner: Prabhav K
 *
 * DESIGN NOTE — where forwarding happens:
 * Comm_OnRxByte() (the ISR side) only parses bytes and validates the
 * checksum. It does NOT call Comm_OnHazardPacket() directly. Instead it
 * sets a "packet ready" flag and Comm_Poll() (called from main()) invokes
 * the callback. This is the "through the main loop" option from the
 * brief: it keeps node_app's state-machine logic (which touches several
 * modules: Sensing, Alert, Log) out of interrupt context, so a slow
 * teammate's function can never turn into a missed UART byte on the next
 * hop. The cost is at most one main-loop iteration of extra latency,
 * which is small as long as main() doesn't block elsewhere — confirm
 * this against Gunadeep's measured latency in the latency budget.
 *
 * Each node uses two relay UARTs plus a third UART for logging (Roshan):
 *   RELAY_RX_HUART  receives from the PREVIOUS node  (relay-in)
 *   RELAY_TX_HUART  transmits to the NEXT node         (relay-out)
 * In Renode, Gunadeep connects RELAY_TX of node k to RELAY_RX of node k+1
 * through one UART hub per link. Which USART peripherals these are is
 * fixed in the UART and pin map (pins.h).
 */
#include "comm.h"
#include "stm32f4xx_hal.h"   /* swap for the matching HAL header if the board changes */
#include <string.h>

/* ------------------------------------------------------------------ */
/* Board wiring — the ONE place this file depends on a specific UART  */
/* ------------------------------------------------------------------ */
extern UART_HandleTypeDef RELAY_RX_HUART;   /* relay-in,  defined/initialised in main.c */
extern UART_HandleTypeDef RELAY_TX_HUART;   /* relay-out, defined/initialised in main.c */

/* ------------------------------------------------------------------ */
/* Internal state                                                     */
/* ------------------------------------------------------------------ */
typedef enum {
    RX_WAIT_SOF = 0,
    RX_COLLECT
} RxState;

static volatile RxState   s_rxState;
static volatile uint8_t   s_rxBuf[COMM_PKT_LEN];
static volatile uint8_t   s_rxCount;
static volatile uint32_t  s_rxLastByteTick;   /* HAL_GetTick() at last byte received */
/* Validated packets wait here (src, seq, sev, hops) until Comm_Poll() hands
 * them to node_app. A queue, not a single flag, so two packets arriving
 * back to back cannot overwrite each other before the main loop runs. */
#define COMM_RX_QUEUE_LEN 4u
static volatile uint8_t   s_rxQ[COMM_RX_QUEUE_LEN][4];
static volatile uint8_t   s_rxQHead, s_rxQTail, s_rxQCount;
static volatile uint32_t  s_rxDroppedCount;   /* valid packets lost: RX queue was full */

static uint8_t  s_txQueue[COMM_TX_QUEUE_LEN][COMM_PKT_LEN];
static volatile uint8_t s_txHead, s_txTail, s_txCount;
static volatile uint8_t s_txBusy;

static volatile uint32_t s_errCount;
static volatile uint32_t s_timeoutCount;
static volatile uint32_t s_droppedTxCount;

/* Single-byte holding cells used with HAL's interrupt-driven API */
static uint8_t s_rxByteHW;
static uint8_t s_txByteHW;
static uint8_t s_txFrameOut[COMM_PKT_LEN];
static uint8_t s_txFrameIdx;
static uint8_t s_txFrameLen;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */
static uint8_t Checksum(uint8_t src, uint8_t seq, uint8_t sev, uint8_t hops)
{
    return (uint8_t)(src ^ seq ^ sev ^ hops);
}

static void BuildPacket(uint8_t *out, uint8_t src, uint8_t seq, uint8_t sev, uint8_t hops)
{
    out[0] = COMM_SOF;
    out[1] = src;
    out[2] = seq;
    out[3] = sev;
    out[4] = hops;
    out[5] = Checksum(src, seq, sev, hops);
}

static void RxReset(void)
{
    s_rxState = RX_WAIT_SOF;
    s_rxCount = 0;
}

/* A frame failed its checksum. The real frame may have started at a later
 * 0xAA inside the bytes we already collected (a stray 0xAA earlier in the
 * stream made us start too soon). Look for it, keep those bytes, and carry
 * on collecting from there instead of throwing the real frame away. */
static void ResyncAfterBadFrame(void)
{
    uint8_t i;
    for (i = 1; i < COMM_PKT_LEN; i++) {
        if (s_rxBuf[i] == COMM_SOF) {
            break;
        }
    }
    if (i < COMM_PKT_LEN) {
        uint8_t n = (uint8_t)(COMM_PKT_LEN - i);
        for (uint8_t k = 0; k < n; k++) {
            s_rxBuf[k] = s_rxBuf[i + k];
        }
        s_rxCount = n;
        s_rxState = RX_COLLECT;
    } else {
        RxReset();
    }
}

/* Kick off transmission of the frame currently sitting at the tail
 * of the queue. Called with the queue known to be non-empty. */
static void StartNextTx(void)
{
    memcpy(s_txFrameOut, s_txQueue[s_txTail], COMM_PKT_LEN);
    s_txFrameIdx = 0;
    s_txFrameLen = COMM_PKT_LEN;
    s_txBusy = 1;
    s_txByteHW = s_txFrameOut[s_txFrameIdx];
    HAL_UART_Transmit_IT(&RELAY_TX_HUART, &s_txByteHW, 1);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
void Comm_Init(void)
{
    s_rxState = RX_WAIT_SOF;
    s_rxCount = 0;
    s_rxQHead = s_rxQTail = s_rxQCount = 0;
    s_rxDroppedCount = 0;
    s_rxLastByteTick = 0;

    s_txHead = s_txTail = s_txCount = 0;
    s_txBusy = 0;

    s_errCount = 0;
    s_timeoutCount = 0;
    s_droppedTxCount = 0;

    /* Arm the first single-byte receive. From here on, every completed
     * receive re-arms itself in HAL_UART_RxCpltCallback(). */
    HAL_UART_Receive_IT(&RELAY_RX_HUART, &s_rxByteHW, 1);
}

uint8_t Comm_SendHazard(uint8_t src, uint8_t seq, uint8_t sev, uint8_t hops)
{
    uint8_t queued = 0;

    __disable_irq();
    if (s_txCount < COMM_TX_QUEUE_LEN) {
        BuildPacket(s_txQueue[s_txHead], src, seq, sev, hops);
        s_txHead = (uint8_t)((s_txHead + 1) % COMM_TX_QUEUE_LEN);
        s_txCount++;
        queued = 1;

        if (!s_txBusy) {
            StartNextTx();
        }
    } else {
        s_droppedTxCount++;
    }
    __enable_irq();

    return queued;
}

void Comm_OnRxByte(uint8_t byte)
{
    /* Runs in ISR context: keep this short, no calls into node_app here. */
    switch (s_rxState) {
        case RX_WAIT_SOF:
            if (byte == COMM_SOF) {
                s_rxBuf[0] = byte;
                s_rxCount = 1;
                s_rxState = RX_COLLECT;
                s_rxLastByteTick = HAL_GetTick();
            }
            /* any other byte while waiting for SOF is silently discarded */
            break;

        case RX_COLLECT:
            s_rxBuf[s_rxCount] = byte;
            s_rxCount++;
            s_rxLastByteTick = HAL_GetTick();

            if (s_rxCount == COMM_PKT_LEN) {
                uint8_t expected = Checksum(s_rxBuf[1], s_rxBuf[2], s_rxBuf[3], s_rxBuf[4]);
                if (expected == s_rxBuf[5]) {
                    if (s_rxQCount < COMM_RX_QUEUE_LEN) {
                        s_rxQ[s_rxQHead][0] = s_rxBuf[1];
                        s_rxQ[s_rxQHead][1] = s_rxBuf[2];
                        s_rxQ[s_rxQHead][2] = s_rxBuf[3];
                        s_rxQ[s_rxQHead][3] = s_rxBuf[4];
                        s_rxQHead = (uint8_t)((s_rxQHead + 1) % COMM_RX_QUEUE_LEN);
                        s_rxQCount++;           /* Comm_Poll() will dispatch it */
                    } else {
                        s_rxDroppedCount++;     /* main loop too slow: count it */
                    }
                    RxReset();
                } else {
                    s_errCount++;
                    ResyncAfterBadFrame();
                }
            }
            break;

        default:
            RxReset();
            break;
    }
}

void Comm_Poll(void)
{
    /* 1) Dispatch every validated packet to node_app, oldest first. The
     *    callback runs outside the critical section, so a slow node_app
     *    never blocks the UART interrupt. */
    for (;;) {
        uint8_t src = 0, seq = 0, sev = 0, hops = 0, have = 0;

        __disable_irq();
        if (s_rxQCount > 0) {
            src  = s_rxQ[s_rxQTail][0];
            seq  = s_rxQ[s_rxQTail][1];
            sev  = s_rxQ[s_rxQTail][2];
            hops = s_rxQ[s_rxQTail][3];
            s_rxQTail = (uint8_t)((s_rxQTail + 1) % COMM_RX_QUEUE_LEN);
            s_rxQCount--;
            have = 1;
        }
        __enable_irq();

        if (!have) {
            break;
        }
        Comm_OnHazardPacket(src, seq, sev, hops);
    }

    /* 2) RX timeout: a frame that started but stalled for too long. */
    __disable_irq();
    uint8_t  stalled   = (s_rxState == RX_COLLECT);
    uint32_t lastByte  = s_rxLastByteTick;
    __enable_irq();

    if (stalled && (HAL_GetTick() - lastByte > COMM_RX_TIMEOUT_MS)) {
        __disable_irq();
        RxReset();
        __enable_irq();
        s_timeoutCount++;
    }
}

uint32_t Comm_GetErrorCount(void)     { return s_errCount; }
uint32_t Comm_GetTimeoutCount(void)   { return s_timeoutCount; }
uint32_t Comm_GetDroppedTxCount(void) { return s_droppedTxCount; }
uint32_t Comm_GetRxDroppedCount(void) { return s_rxDroppedCount; }

/* ------------------------------------------------------------------ */
/* HAL interrupt callbacks                                            */
/* ------------------------------------------------------------------ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == RELAY_RX_HUART.Instance) {
        Comm_OnRxByte(s_rxByteHW);
        HAL_UART_Receive_IT(&RELAY_RX_HUART, &s_rxByteHW, 1);  /* re-arm */
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == RELAY_TX_HUART.Instance) {
        s_txFrameIdx++;
        if (s_txFrameIdx < s_txFrameLen) {
            s_txByteHW = s_txFrameOut[s_txFrameIdx];
            HAL_UART_Transmit_IT(&RELAY_TX_HUART, &s_txByteHW, 1);
        } else {
            /* Frame finished: pop the queue and start the next one, if any. */
            s_txTail = (uint8_t)((s_txTail + 1) % COMM_TX_QUEUE_LEN);
            s_txCount--;
            if (s_txCount > 0) {
                StartNextTx();
            } else {
                s_txBusy = 0;
            }
        }
    }
}
