/*
 * [File Name]     hardware_init.c
 * [Platform]      FRDM-K22F
 * [Project]       lab1_project
 * [Version]       1.00
 * [Author]        r59716
 * [Date]          06/10/2015
 * [Language]      'C'
 * [History]       1.00 - Original Release
 *
 */

//-----------------------------------------------------------------------
// KSDK Includes
//-----------------------------------------------------------------------
#include "board.h"
#include "pin_mux.h"
#include "fsl_clock_manager.h"
#include "fsl_debug_console.h"
#include "fsl_pmc_hal.h"

//-----------------------------------------------------------------------
// Hardware Initialization
//-----------------------------------------------------------------------
void hardware_init(void)
{
    /* enable clock for PORTs */
    CLOCK_SYS_EnablePortClock(PORTA_IDX);
    CLOCK_SYS_EnablePortClock(PORTB_IDX);
    CLOCK_SYS_EnablePortClock(PORTC_IDX);
    CLOCK_SYS_EnablePortClock(PORTD_IDX);
    CLOCK_SYS_EnablePortClock(PORTE_IDX);

    if(PMC_HAL_GetAckIsolation(PMC_BASE_PTR) != 0)
    {
        PMC_HAL_ClearAckIsolation(PMC_BASE_PTR);
    }

    /* Init board clock */
    BOARD_ClockInit();
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
