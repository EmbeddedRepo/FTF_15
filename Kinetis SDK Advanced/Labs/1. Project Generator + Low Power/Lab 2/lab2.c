/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

///////////////////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>

#include "fsl_os_abstraction.h"
#include "fsl_interrupt_manager.h"
#include "fsl_power_manager.h"
#include "fsl_llwu_hal.h"
#include "fsl_smc_hal.h"
#include "fsl_clock_manager.h"
#include "fsl_debug_console.h"
#include "fsl_sim_hal.h"
#include "fsl_misc_utilities.h"
#include "fsl_uart_driver.h"
#include "fsl_rtc_hal.h"

#include "fsl_device_registers.h"

#include "board.h"


///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////

// Enum with supported power modes.
typedef enum demo_power_modes
{
    kDemoMin = 'A' -1,
    kDemoRun = 'A',
	kDemoHsRun,
    kDemoWait ,
    kDemoStop,
    kDemoVlpr,
    kDemoVlpw,
    kDemoVlps,
    kDemoLls3,
	kDemoVlls0,
    kDemoMax
} demo_power_modes_t;

extern const clock_manager_user_config_t g_defaultClockConfigRun;
extern const clock_manager_user_config_t g_defaultClockConfigVlpr;
extern const clock_manager_user_config_t g_defaultClockConfigHsrun;

//******************************************************************************
//* Interrupt handlers used to wake the device from the various low power modes.
//******************************************************************************
void LLWU_IRQHandler(void)
{
    // Clear the interrupt flag.
    LLWU_HAL_ClearExternalPinWakeupFlag(LLWU, BOARD_SW_LLWU_EXT_PIN);
}

void PORTC_IRQHandler(void)
{
	// Clear the interrupt flag.
    PORT_HAL_ClearPortIntFlag(PORTC);
}

/*******************************************************************************
 * Callback function for Clock Manager.
 ******************************************************************************/
clock_manager_error_code_t clockManagerCallback(clock_notify_struct_t *notify,
                                    			void* callbackData)
{
    clock_manager_error_code_t ret = kClockManagerError;

    switch (notify->notifyType)
    {
        case kClockManagerNotifyBefore:
            DbgConsole_DeInit();
            ret = kClockManagerSuccess;
            break;
            
        case kClockManagerNotifyRecover:
        case kClockManagerNotifyAfter:
        	DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_LOW_POWER_UART_BAUD,
        	            	kDebugConsoleUART);
        	ret = kClockManagerSuccess;
            break;
            
        default:
            break;
    }
    
    return ret;
}

/*******************************************************************************
 * Callback function for Power Manager.
 ******************************************************************************/
power_manager_error_code_t powerManagerCallback(power_manager_notify_struct_t * notify,
                                     	 	 	power_manager_callback_data_t * dataPtr)
{
    power_manager_error_code_t ret = kPowerManagerError;
    volatile bool isLastByteTranmistComplete = false;

    switch (notify->notifyType)
    {
        case kPowerManagerNotifyBefore:
        	// Wait for any pending UART data to go out.
            do
            {
                isLastByteTranmistComplete = UART_HAL_GetStatusFlag(BOARD_DEBUG_UART_BASEADDR,
                													kUartTxComplete);
            } while (!isLastByteTranmistComplete);

            // Disable the UART pins.
            PORT_HAL_SetMuxMode(PORTE, 0u, kPortPinDisabled);
            PORT_HAL_SetMuxMode(PORTE, 1u, kPortPinDisabled);

            ret = kPowerManagerSuccess;
            break;

        case kPowerManagerNotifyAfter:

        	// Re-enable the UART pins.
        	PORT_HAL_SetMuxMode(PORTE, 0u, kPortMuxAlt3);
        	PORT_HAL_SetMuxMode(PORTE, 1u, kPortMuxAlt3);

            ret = kPowerManagerSuccess;
            break;

        default:
            break;
    }

    return ret;
}

/*******************************************************************************
 * Main function for application.
 ******************************************************************************/
void lab2_power(void)
{
    demo_power_modes_t testVal = kDemoRun;
    volatile uint8_t powerMode;
    uint8_t clockManagerMode = CLOCK_RUN;
    uint32_t freq = 0;

    //*******************************************************
	//* Step 1: Initialize the Clock Manager  configurations.
	//*******************************************************

    // Create list of supported clock configurations.  These are taken from the
    // board.c file and will be passed into CLOCK_SYS_Init().
    clock_manager_user_config_t const *clockConfigs[] =
    {
        NULL,
        &g_defaultClockConfigVlpr,
        &g_defaultClockConfigRun,
        &g_defaultClockConfigHsrun,
    };

    //*****************************************************
	//* Step 2: Configure Clock Manager callback function.
	//*****************************************************

    // Clock Manager callback function.
    clock_manager_callback_user_config_t clockManagerCallbackCfg =
    {
        .callback     = clockManagerCallback,
        .callbackType = kClockManagerCallbackBeforeAfter,
        .callbackData = NULL
    };

    clock_manager_callback_user_config_t *clockCallbacks[] =
    {
        &clockManagerCallbackCfg
    };

    //*****************************************************
	//* Step 3: Initialize the Clock Manager.
	//*****************************************************

    // Pass in configuration and callback data to Clock Manager.
	CLOCK_SYS_Init(clockConfigs,
				   CLOCK_NUMBER_OF_CONFIGURATIONS,
				   clockCallbacks,
				   ARRAY_SIZE(clockCallbacks));

	// Set to RUN mode.
	CLOCK_SYS_UpdateConfiguration(CLOCK_RUN, kClockManagerPolicyForcible);

    //*****************************************************
    //* Step 4: Set up supported power mode structures.
    //*****************************************************

    const power_manager_user_config_t runConfig =
    {
        .mode = kPowerManagerRun,
        .sleepOnExitValue = false,
    };

    power_manager_user_config_t hsrunConfig = runConfig;
    hsrunConfig.mode = kPowerManagerHsrun;

    power_manager_user_config_t waitConfig  = runConfig;
    waitConfig.mode = kPowerManagerWait;

    power_manager_user_config_t stopConfig  = runConfig;
    stopConfig.mode = kPowerManagerStop;

    power_manager_user_config_t vlprConfig   = runConfig;
    vlprConfig.mode = kPowerManagerVlpr;

    power_manager_user_config_t vlpwConfig	= runConfig;
    vlpwConfig.mode = kPowerManagerVlpw;

    power_manager_user_config_t vlpsConfig  = runConfig;
    vlpsConfig.mode = kPowerManagerVlps;

    power_manager_user_config_t lls3Config   = runConfig;
    lls3Config.mode = kPowerManagerLls3;

    power_manager_user_config_t vlls0Config   = runConfig;
    vlls0Config.mode = kPowerManagerVlls0;

    //***********************************************************
    //* Step 5: Configure managed power configurations structure.
    //***********************************************************

    // Create the list of supported modes to pass into the Power Manager.
    power_manager_user_config_t const *powerConfigs[] =
    {
    	&runConfig,
		&hsrunConfig,
		&waitConfig,
		&stopConfig,
		&vlprConfig,
		&vlpwConfig,
		&vlpsConfig,
		&lls3Config,
		&vlls0Config
    };

    //*****************************************************
	//* Step 6: Configure Power Manager callback function.
	//*****************************************************

    // Initializes callback configuration structure for the Power Manager.
    power_manager_callback_user_config_t powerManagerCallbackCfg =
    {
		.callback  	  = powerManagerCallback,
		.callbackType = kPowerManagerCallbackBeforeAfter,
		.callbackData = NULL
    };

    // Initializes array of pointers to power manager callbacks.
    power_manager_callback_user_config_t *powerCallbacks[] =
    {
    	&powerManagerCallbackCfg
    };

    //*****************************************************
    //* Step 7: Initialize the Power Manager.
    //*****************************************************

    // Pass power configurations and callback info to Power Manager.
    POWER_SYS_Init(powerConfigs,
    			   sizeof(powerConfigs) / sizeof(power_manager_user_config_t *),
				   powerCallbacks,
				   ARRAY_SIZE(powerCallbacks));

    // Initialize system to RUN mode.
    powerMode = kDemoRun - kDemoMin - 1;
    POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

    //**************************************************************
    //* Step 8: Configure the rest of the chip for this application.
    //**************************************************************

    // Configure pin mux for UART functionality.
    configure_uart_pins(BOARD_DEBUG_UART_INSTANCE);

    // Initializes GPIO driver for LEDs and buttons
    GPIO_DRV_Init(switchPins, ledPins);

    // Initialize the debug UART console.
    DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_LOW_POWER_UART_BAUD,
    				kDebugConsoleUART);

    // Enable PORTC clock for GPIO/LLWU interrupt.
    CLOCK_SYS_EnablePortClock(2);

    // Enables falling edge interrupt for switch SW2.
    PORT_HAL_SetMuxMode(BOARD_SW_LLWU_BASE, BOARD_SW_LLWU_PIN, kPortMuxAsGpio);
    PORT_HAL_SetPinIntMode(BOARD_SW_LLWU_BASE, BOARD_SW_LLWU_PIN, kPortIntFallingEdge);

    // Enable GPIO interrupt.
    INT_SYS_EnableIRQ(PORTC_IRQn);

    // Configure the LLWU.
    LLWU_HAL_ClearExternalPinWakeupFlag(LLWU, BOARD_SW_LLWU_EXT_PIN);
    LLWU_HAL_SetExternalInputPinMode(LLWU, kLlwuExternalPinFallingEdge, BOARD_SW_LLWU_EXT_PIN);

    // Enable LLWU interrupt.
    INT_SYS_EnableIRQ(LLWU_IRQn);

    // Main loop.
    while (1)
    {
    	// Get the system clock frequency and current clock configuration to print out.
        CLOCK_SYS_GetFreq(kCoreClock, &freq);
        clockManagerMode = CLOCK_SYS_GetCurrentConfiguration();

        PRINTF("\n\r####################  Power Manager Demo ####################\n\n\r");
        PRINTF("    Core Clock   = %d MHz \n\r", freq / 1000000);
        PRINTF("    Current Mode = ");

        switch(clockManagerMode)
        {
			case CLOCK_RUN:
				PRINTF("RUN\n\r");
				break;
			case CLOCK_VLPR:
				PRINTF("VLPR\n\r");
				break;
			case CLOCK_HSRUN:
				PRINTF("HSRUN\n\r");
				break;
        }

        PRINTF("\n\rSelect the desired operation \n\n\r");
        PRINTF("Press  %c for enter: RUN   - Normal RUN mode\n\r", kDemoRun);
        PRINTF("Press  %c for enter: HSRUN - High Speed RUN mode\n\r", kDemoHsRun);
        PRINTF("Press  %c for enter: Wait  - Wait mode\n\r", kDemoWait);
        PRINTF("Press  %c for enter: Stop  - Stop mode\n\r", kDemoStop);
        PRINTF("Press  %c for enter: VLPR  - Very Low Power Run mode\n\r", kDemoVlpr);
        PRINTF("Press  %c for enter: VLPW  - Very Low Power Wait mode\n\r", kDemoVlpw);
        PRINTF("Press  %c for enter: VLPS  - Very Low Power Stop mode\n\r", kDemoVlps);
        PRINTF("Press  %c for enter: LLS3  - Low Leakage Stop mode\n\r", kDemoLls3);
        PRINTF("Press  %c for enter: VLLS0 - Very Low Leakage Stop mode 0\n\r", kDemoVlls0);
        //PRINTF("Press  %c for enter: VLLS3 - Very Low Leakage Stop mode 3\n\r", kDemoVlls3);

        PRINTF("\n\rWaiting for key press...\n\r\n\r");

        // Wait for user input.
        testVal = (demo_power_modes_t)GETCHAR();

        if ((testVal >= 'a') && (testVal <= 'z'))
        {
            testVal -= 'a' - 'A';
        }

        if (testVal > kDemoMin && testVal < kDemoMax)
        {
        	// Obtain Power Manager readable version of the value passed in.
        	powerMode = testVal - kDemoMin - 1;

        	// Switch to selected mode.
            switch (testVal)
            {
				case kDemoRun:

					if (POWER_SYS_GetCurrentMode() == kPowerManagerRun)
					{
						PRINTF("Run mode already active.\n\r");
						break;
					}

					// Update the power configuration.
					POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

					// Update the clock configuration.
					CLOCK_SYS_UpdateConfiguration(CLOCK_RUN, kClockManagerPolicyAgreement);

					break;

				case kDemoHsRun:

					if (POWER_SYS_GetCurrentMode() == kPowerManagerHsrun)
					{
						PRINTF("High Speed Run mode already active.\n\r");
						break;
					}

					// Update the power configuration.
					POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

					// Update the clock configuration.
					CLOCK_SYS_UpdateConfiguration(CLOCK_HSRUN, kClockManagerPolicyAgreement);

					break;

                case kDemoWait:

                    if (POWER_SYS_GetCurrentMode() == kPowerManagerVlpr)
                    {
                        PRINTF("Cannot go from VLPR to WAIT directly...\n\r");
                        break;
                    }
                    if (POWER_SYS_GetCurrentMode() == kPowerManagerHsrun)
                    {
                        PRINTF("Cannot go from HSRUN to WAIT directly...\n\r");
                        break;
                    }

                    PRINTF("Entering WAIT mode. Press SW2 to wake...\n\r");

                    // Update the power configuration.
                    POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

                    break;

                case kDemoStop:

                    if (POWER_SYS_GetCurrentMode() == kPowerManagerVlpr)
                    {
                        PRINTF("Cannot go from VLPR to STOP directly...\n\r");
                        break;
                    }
                    if (POWER_SYS_GetCurrentMode() == kPowerManagerHsrun)
                    {
                        PRINTF("Cannot go from HSRUN to STOP directly...\n\r");
                        break;
                    }

                    PRINTF("Entering STOP mode. Press SW2 to wake...\n\r");

                    // Update the power configuration.
                    POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

                    // Update the clock configuration.
                    CLOCK_SYS_UpdateConfiguration(clockManagerMode, kClockManagerPolicyAgreement);

                    break;

                case kDemoVlpr:

                    if (POWER_SYS_GetCurrentMode() == kPowerManagerHsrun)
                    {
                        PRINTF("Cannot go from HSRUN to VLPR directly...\n\r");
                        break;
                    }

                    if(POWER_SYS_GetCurrentMode() != kPowerManagerVlpr)
                    {
                    	// Update the clock configuration PRIOR to entering VLPR.
                    	CLOCK_SYS_UpdateConfiguration(CLOCK_VLPR, kClockManagerPolicyAgreement);

                    	// Update the power configuration.
                        POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);
                    }
                    else
                    {
                        PRINTF("Very Low Power Run mode already active.\n\r");
                    }

                    break;

                case kDemoVlpw:

                    if (POWER_SYS_GetCurrentMode() == kPowerManagerRun)
                    {
                        PRINTF("Cannot go from RUN to VLPW directly...\n\r");
                        break;
                    }

                    if (POWER_SYS_GetCurrentMode() == kPowerManagerHsrun)
                    {
                        PRINTF("Cannot go from HSRUN to VLPW directly...\n\r");
                        break;
                    }

                    PRINTF("Entering VLPW mode. Press SW2 to wake...\n\r");

                    // Update the power configuration.
                    POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

                    break;

                case kDemoVlps:

                    if (POWER_SYS_GetCurrentMode() == kPowerManagerHsrun)
                    {
                        PRINTF("Cannot go from HSRUN to VLPS directly...\n\r");
                        break;
                    }

                    PRINTF("Entering VLPS mode. Press SW2 to wake...\n\r");

                    // Update the power configuration.
                    POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

                    // If we entered via RUN mode, restore clock settings.
                    if (POWER_SYS_GetCurrentMode() == kPowerManagerRun)
					{
                    	CLOCK_SYS_UpdateConfiguration(clockManagerMode, kClockManagerPolicyAgreement);
					}

                    break;

                case kDemoLls3:

                    if (POWER_SYS_GetCurrentMode() == kPowerManagerHsrun)
                    {
                        PRINTF("Cannot go from HSRUN to LLSx directly...\n\r");
                        break;
                    }

                    PRINTF("Entering LLS3 mode. Press SW2 to wake...\n\r");

                    // Update the power configuration.
                    POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

                    // Check the mode active mode prior to LLS3 entry.
                    if(POWER_SYS_GetCurrentMode() != kPowerManagerVlpr)
                    {
                        CLOCK_SYS_UpdateConfiguration(clockManagerMode, kClockManagerPolicyAgreement);
                    }

                    break;

                case kDemoVlls0:

                    if (POWER_SYS_GetCurrentMode() == kPowerManagerHsrun)
                    {
                        PRINTF("Cannot go from HSRUN to VLLSx directly...\n\r");
                        break;
                    }

                    PRINTF("Press SW2 to wake. VLLSx wake goes through RESET sequence.\n\r");

                    // Update the power configuration.
                    POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

                    break;

                default:
                    PRINTF("Bad value.");
                    break;
            }

            PRINTF("\n\rNext loop\n\r");
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// EOF
///////////////////////////////////////////////////////////////////////////////
