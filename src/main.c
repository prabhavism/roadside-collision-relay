/* main.c (skeleton)
 * Shows the init order your modules need. The HAL_Init(),
 * SystemClock_Config() and MX_xxx_Init() calls are the usual
 * CubeMX-generated boilerplate — fill those in once the board is
 * confirmed; everything else here is what actually matters for the
 * relay chain to work correctly.
 *
 * NOTE: once node_app.c is in the build, remove test_comm_main.c (or
 * its TestMain()/Comm_OnHazardPacket from the build) — node_app.c now
 * provides the real Comm_OnHazardPacket(), and having two definitions
 * will fail to link.
 */
#include "stm32f4xx_hal.h"
#include "comm.h"
#include "node_app.h"
#include "sensing.h"
#include "alert.h"
#include "log.h"

UART_HandleTypeDef RELAY_RX_HUART;  /* relay-in  (from previous node) */
UART_HandleTypeDef RELAY_TX_HUART;  /* relay-out (to next node)       */
UART_HandleTypeDef LOG_HUART;     /* MX_USARTy_Init() fills this in   */

int main(void)
{
    HAL_Init();
    /* SystemClock_Config(); */
    /* MX_GPIO_Init(); */
    /* MX_USARTx_Init(&RELAY_RX_HUART, ...);  MX_USARTz_Init(&RELAY_TX_HUART, ...); */
    /* MX_USARTy_Init(&LOG_HUART, ...); */
    /* MX_ADC_Init();  MX_TIMx_Init();  -- Rohan's sampling timer/ADC   */

    /* Init order matters: Comm_Init() arms the UART RX interrupt, so
     * it should come after the UART peripherals themselves are set
     * up, and node_app should be initialised last since it's the one
     * that ties everything together. */
    Log_Init();
    Alert_Init();
    Sensing_Init();
    Comm_Init();
    Node_App_Init();

    while (1) {
        Node_Run();     /* polls comm, drives the state machine        */
        /* Sensing_OnTick() runs from the 1 ms timer ISR, not here.    */
    }
}
