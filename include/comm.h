/* comm.h
 * Core Relay & Control — node-to-node hazard relay over UART.
 * Owner: Prabhav K
 *
 * Packet format (fixed, from the Interface Sheet):
 *   byte 0: 0xAA   (start of frame)
 *   byte 1: SRC    (originating node ID, 1..3)
 *   byte 2: SEQ    (sequence number, increases per new hazard at the source)
 *   byte 3: SEV    (severity, 1..3)
 *   byte 4: HOPS   (number of relays so far, starts at 0)
 *   byte 5: CHK    (XOR of SRC, SEQ, SEV, HOPS)
 *
 * Wiring: TX of node k -> RX of node k+1.
 * Link:   115200 baud, 8N1, interrupt-driven (no blocking calls in the hazard path).
 */
#ifndef COMM_H
#define COMM_H

#include <stdint.h>

#define COMM_PKT_LEN        6u
#define COMM_SOF            0xAAu
#define COMM_TX_QUEUE_LEN    4u      /* max packets queued for transmit at once   */
#define COMM_RX_TIMEOUT_MS   5u      /* reset the parser if a frame stalls this long */

/* --- lifecycle --- */
void Comm_Init(void);

/* Call once per main-loop iteration. Services the TX queue and checks
 * for a stalled RX frame (timeout). Cheap — safe to call every loop. */
void Comm_Poll(void);

/* --- sending --- */
/* Non-blocking: queues the packet and returns immediately.
 * Returns 1 if queued, 0 if the queue was full (caller should count this
 * as a soft error — see Comm_GetDroppedTxCount). */
uint8_t Comm_SendHazard(uint8_t src, uint8_t seq, uint8_t sev, uint8_t hops);

/* --- receiving --- */
/* Call this from the UART RX interrupt handler with exactly one byte.
 * Must be short: no printf, no loops, no blocking. */
void Comm_OnRxByte(uint8_t byte);

/* --- diagnostics (for scenario S2) --- */
uint32_t Comm_GetErrorCount(void);      /* bad checksum / framing errors seen   */
uint32_t Comm_GetTimeoutCount(void);    /* RX frames that stalled and were reset */
uint32_t Comm_GetDroppedTxCount(void);  /* packets dropped because TX queue was full */

/* --- the callback you (Prabhav) implement in node_app.c ---
 * Called from Comm_Poll() context (NOT from the ISR) once a full, valid
 * packet has been received. This keeps node_app's logic out of interrupt
 * context, which is the safer default — see the design note in comm.c. */
void Comm_OnHazardPacket(uint8_t src, uint8_t seq, uint8_t sev, uint8_t hops);

#endif /* COMM_H */