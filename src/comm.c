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
 * Retarget for your actual board/pins by changing RELAY_HUART below
 * (and the corresponding init in main.c / config.h) once Gunadeep
 * confirms the Pin Map.
 */
#include "comm.h"
#include "stm32f4xx_hal.h"   /* swap for the matching HAL header if the board changes */
#include <string.h>

/* ------------------------------------------------------------------ */
/* Board wiring — the ONE place this file depends on a specific UART  */
/* ------------------------------------------------------------------ */
extern UART_HandleTypeDef RELAY_HUART;   /* defined/initialised in main.c */

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
static volatile uint8_t   s_rxPacketReady;    /* set by ISR, cleared by Comm_Poll    */

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

/* Kick off transmission of the frame currently sitting at the tail
 * of the queue. Called with the queue known to be non-empty. */
static void StartNextTx(void)
{
    memcpy(s_txFrameOut, s_txQueue[s_txTail], COMM_PKT_LEN);
    s_txFrameIdx = 0;
    s_txFrameLen = COMM_PKT_LEN;
    s_txBusy = 1;
    s_txByteHW = s_txFrameOut[s_txFrameIdx];
    HAL_UART_Transmit_IT(&RELAY_HUART, &s_txByteHW, 1);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
void Comm_Init(void)
{
    s_rxState = RX_WAIT_SOF;
    s_rxCount = 0;
    s_rxPacketReady = 0;
    s_rxLastByteTick = 0;

    s_txHead = s_txTail = s_txCount = 0;
    s_txBusy = 0;

    s_errCount = 0;
    s_timeoutCount = 0;
    s_droppedTxCount = 0;

    /* Arm the first single-byte receive. From here on, every completed
     * receive re-arms itself in HAL_UART_RxCpltCallback(). */
    HAL_UART_Receive_IT(&RELAY_HUART, &s_rxByteHW, 1);
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
                    s_rxPacketReady = 1;   /* Comm_Poll() will dispatch it */
                } else {
                    s_errCount++;
                    /* Resynchronise: a bad frame doesn't necessarily mean
                     * the NEXT 0xAA we saw was real garbage, so just go
                     * back to hunting for SOF. */
                }
                RxReset();
            }
            break;

        default:
            RxReset();
            break;
    }
}

void Comm_Poll(void)
{
    /* 1) Dispatch a fully-received, checksum-valid packet to node_app. */
    if (s_rxPacketReady) {
        uint8_t src, seq, sev, hops;

        __disable_irq();
        src  = s_rxBuf[1];
        seq  = s_rxBuf[2];
        sev  = s_rxBuf[3];
        hops = s_rxBuf[4];
        s_rxPacketReady = 0;
        __enable_irq();

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

/* ------------------------------------------------------------------ */
/* HAL interrupt callbacks                                            */
/* ------------------------------------------------------------------ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == RELAY_HUART.Instance) {
        Comm_OnRxByte(s_rxByteHW);
        HAL_UART_Receive_IT(&RELAY_HUART, &s_rxByteHW, 1);  /* re-arm */
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == RELAY_HUART.Instance) {
        s_txFrameIdx++;
        if (s_txFrameIdx < s_txFrameLen) {
            s_txByteHW = s_txFrameOut[s_txFrameIdx];
            HAL_UART_Transmit_IT(&RELAY_HUART, &s_txByteHW, 1);
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