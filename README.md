\# Roadside Chain-Collision Early-Warning Relay for Highways and Tunnels

23CSE304 Embedded Systems — Course Project



\## Team



| Name | Roll Number | Module |

|---|---|---|

| Rohan S | AM.SC.U4CSE24349 | Sensing \& Detection |

| Prabhav K | AM.SC.U4CSE24330 | Core Relay \& Control (critical path) |

| Praneeth S | AM.SC.U4CSE24351 | Alert \& Timekeeping |

| Gunadeep | AM.SC.U4CSE24334 | Simulation \& Topology |

| Roshan P | AM.SC.U4CSE24342 | Integration \& Logging |



\## What this is



Three simulated roadside nodes, each an ARM Cortex-M microcontroller, detect

sudden vehicle deceleration (ADC input, simulated via a potentiometer in

Wokwi) and relay a hazard warning to the next node over UART within a hard

real-time latency budget (target: 10 ms, node 1 to node 3). Each node

triggers a local beacon, logs the event with a millisecond-resolution RTC

timestamp, and forwards the warning unless it is the last node in the chain.



Target SDGs: SDG 11 (Sustainable Cities and Communities), SDG 3 (Good Health

and Well-being).



\## Folder layout



```

src/        sensing.c, comm.c, alert.c, log.c, node\_app.c, main.c

include/    matching .h files, pins.h, config.h

wokwi/      diagram.json, wokwi.toml

docs/       design notes, screenshots, latency captures, results —

&#x20;           one subfolder per person: docs/rohan/, docs/prabhav/,

&#x20;           docs/praneeth/, docs/gunadeep/, docs/roshan/

```



\## Interface Sheet (fixed — do not change without updating this file)



| Item | Agreed value | Owner |

|---|---|---|

| Nodes | 3 nodes, IDs 1 to 3. Warning flows 1 → 2 → 3. | Gunadeep |

| Relay link | UART, 115200 baud, 8N1. TX of node \*k\* → RX of node \*k+1\*. | Prabhav |

| Log link | A separate UART to the serial monitor (not the relay UART). | Roshan |

| Packet format | 6 bytes: `0xAA, SRC, SEQ, SEV, HOPS, CHK`. `CHK` = XOR of SRC, SEQ, SEV, HOPS. SEQ increases with each new hazard at the source node. | Prabhav |

| Sampling | Timer interrupt every 1 ms. ADC input from the potentiometer. | Rohan |

| Detection rule | Filtered value above `HAZ\_THRESHOLD` for 5 consecutive samples. Severity 1–3 by magnitude band. | Rohan |

| Beacon | LED, plus buzzer at severity 3. On for 3 s (`BEACON\_MS`), then cooldown. | Praneeth |

| Debug pins | `DBG\_DETECT` toggles on local detection. `DBG\_BEACON` toggles when the beacon turns on. | Rohan, Praneeth |

| Log line format | `\[HH:MM:SS.mmm] N<id> EVENT=<name> SRC=<n> SEQ=<n> SEV=<n>` | Roshan |

| Duplicate rule | Ignore a packet whose `(SRC, SEQ)` pair was already handled. | Prabhav |

| Latency target | Detection at node 1 to beacon at node 3 within 10 ms of simulated time. | Gunadeep |

| Code layout | STM32 HAL only. Function names below. | All |



\### Function names (frozen — all modules build against these)



```

Sensing\_Init, Sensing\_OnTick, Sensing\_HazardDetected, Sensing\_GetSeverity, Sensing\_ClearHazard

Comm\_Init, Comm\_SendHazard, Comm\_OnRxByte, Comm\_Poll, Comm\_GetErrorCount, Comm\_OnHazardPacket

Alert\_Init, Alert\_BeaconOn, Alert\_BeaconOff, Rtc\_Init, Rtc\_GetTimestamp

Log\_Init, Log\_Event

Node\_App\_Init, Node\_Run

```



\## Branch workflow



\- Branch naming: `yourname/module`, e.g. `prabhav/relay-control`.

\- `main` only changes through a pull request. It must always build.

\- Your PR is reviewed by your assigned reviewer (see the review ring in the

&#x20; Team Work Plan) before merging.

\- Pull `main` at least once a day.



\## Build \& run



See `docs/<your-name>/how-to-run.md` (Gunadeep publishes the canonical

version once the build method is confirmed in Week 1).



\## Status



\- \[ ] Repository set up and all five collaborators added

\- \[ ] Stub files published (Roshan)

\- \[ ] Wokwi 3-node diagram working (Gunadeep)

\- \[ ] Each module built and unit-tested

\- \[ ] Integration complete, all three nodes relay correctly

\- \[ ] Report and demo ready



