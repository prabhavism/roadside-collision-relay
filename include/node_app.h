/* node_app.h
 * Core Relay & Control — node state machine.
 * Owner: Prabhav K
 *
 * States: IDLE -> LOCAL_HAZARD | RELAYED_HAZARD -> COOLDOWN -> IDLE
 *
 * This module is the glue that calls into every teammate's module:
 *   Sensing_HazardDetected() / Sensing_GetSeverity() / Sensing_ClearHazard()  (Rohan)
 *   Alert_BeaconOn() / Alert_BeaconOff()                                      (Praneeth)
 *   Log_Event()                                                               (Roshan)
 *   Comm_SendHazard()                                                        (this module, comm.c)
 *
 * Until Roshan's real stub/final files land, this file links against the
 * exact signatures agreed in the Interface Sheet — drop his files in
 * without changing anything here.
 */
#ifndef NODE_APP_H
#define NODE_APP_H

#include <stdint.h>

/* Call once at start-up, after Comm_Init(), Sensing_Init(), Alert_Init(),
 * Log_Init() have all run. */
void Node_App_Init(void);

/* Call every iteration of the main loop. Drives the state machine:
 * checks for a local hazard, lets Comm_Poll() dispatch any received
 * packet (which calls Comm_OnHazardPacket -> this module), and handles
 * the cooldown timer. */
void Node_Run(void);

#endif /* NODE_APP_H */
