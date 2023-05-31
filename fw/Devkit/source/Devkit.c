/*
 * Copyright 2016-2023 NXP
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
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
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

/**
 * @file    Devkit.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "K32L2B31A.h"
#include "fsl_debug_console.h"

#include "fsl_smc.h"
#include "fsl_llwu.h"
#include "fsl_adc16.h"
#include "fsl_lptmr.h"
#include "sf.h"
#include "sf_ol2385.h"
/* definitions and declarations */
#define TIMER_PERIOD (60U * 15U) // length of sleep ( minutes * seconds )
#define DEMO_ADC16_BASE          ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL  26U /* internal temperature sensor, ADC0_SE26 */
/* Define LLWU interrupt handler */
void LLWU_IRQHandler(void)
{
	LLWU_GetInternalWakeupModuleFlag(LLWU, 0);
	LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);
	LPTMR_StopTimer(LPTMR0);
}

/* Define interrupt handler for Low Power Timer */
void LPTMR0_IRQHandler(void)
{
	LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);
	LPTMR_StopTimer(LPTMR0);
}
/*
 * @brief   Application entry point.
 */
int main(void) {
	adc16_config_t adc16ConfigStruct;
	adc16_channel_config_t adc16ChannelConfigStruct;
	status_t powerStatus;

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif
	GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
	while(true){

		// allow VLPR modes (red LED is off)
		SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);

		powerStatus = SMC_SetPowerModeVlpr(SMC);
		if (kStatus_Success != powerStatus)
		{
			while (true)
			{
				{__asm volatile ("nop");}
			}
		}

		// ADC *********************

		/*
		 * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
		 * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
		 * adc16ConfigStruct.enableAsynchronousClock = false;
		 * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
		 * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
		 * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
		 * adc16ConfigStruct.enableHighSpeed = false;
		 * adc16ConfigStruct.enableLowPower = false;
		 * adc16ConfigStruct.enableContinuousConversion = false;
		 */
		ADC16_GetDefaultConfig(&adc16ConfigStruct);

		// enable low power in adc config
		adc16ConfigStruct.enableLowPower = true;

	#ifdef BOARD_ADC_USE_ALT_VREF
		adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
	#endif
		ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
		ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */
	#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
		if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASE))
		{
			PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
		}
		else
		{
			PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
		}
	#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

		adc16ChannelConfigStruct.channelNumber                        = DEMO_ADC16_USER_CHANNEL;
		adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
	#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
		adc16ChannelConfigStruct.enableDifferentialConversion = false;
	#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

		/*
		 When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
		 function, which works like writing a conversion command and executing it. For another channel's conversion,
		 just to change the "channelNumber" field in channel's configuration structure, and call the
		 "ADC16_ChannelConfigure() again.
		*/
		ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
		while (0U == (kADC16_ChannelConversionDoneFlag &
					  ADC16_GetChannelStatusFlags(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP)))
		{
		}

		int32_t currentTemperature = 0;
		uint32_t adcValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP);
		/* Temperature = 25 - (ADCR_T - ADCR_TEMP25) * 100 / ADCR_100M */
		currentTemperature = (int32_t)(25 - ((int32_t)(adcValue << 4) - 15030) * 100 / 3400);

		// convert from integer temperature to
		uint8_t convertedTemperature = currentTemperature * 2 + 128;

		PRINTF("ADC read temperature: %d, sigfox payload would be %X	\r\n", currentTemperature, convertedTemperature);

		// construct message

		uint8_t message[12] = {0};
		message[0] = 0xE8;
		message[1] = convertedTemperature;

		// SIGFOX ******************

		// sf init
		sf_drv_data_t sfDrvData;
		sf_status_t sfResult;
		sfDrvData.drvInstance = 0U;
		sfResult = SF_Init(&sfDrvData, NULL);

		// sf wakeup
		sfResult = SF_WakeUp(&sfDrvData);

		uint8_t buffer[4];
		sfResult = SF_GetSigfoxId(&sfDrvData, buffer);

		// sf tx / send frame
		sf_msg_payload_t sfPayload;
		sfPayload.payloadLen = 12U;
		sfPayload.payload = message;
		PRINTF("Transmitting Sigfox message...\n");
		sfResult = SF_SendFrame(&sfDrvData, &sfPayload);
//		sfResult = sfStatusSuccess;
		if (sfStatusSuccess == sfResult) {
			PRINTF("Message sent successfully.\r\n");
		} else {
			PRINTF("Something went wrong... sf_status %u\r\n", sfResult);
		}

		// sf sleep
		sfResult = SF_Sleep(&sfDrvData);

		// LPTMR *********************

		// wait to show red led is off to show VLPR is entred
		for(uint32_t j = 0; j<400000; j++)
		{
			__asm volatile ("nop");
		}

		/* Configure Low Power Timer (LPTMR) */
		lptmr_config_t lptmrConfig;
		/* LPO is the clock source in the default config (1kHz) with no prescaler */
		LPTMR_GetDefaultConfig(&lptmrConfig);
		lptmrConfig.bypassPrescaler = false; /* Enable prescaler */
		lptmrConfig.value = kLPTMR_Prescale_Glitch_9; /* Prescaler divide 1024 */

		/* Initialize the LPTMR */
		LPTMR_Init(LPTMR0, &lptmrConfig);

		/* Set timer period */
		LPTMR_SetTimerPeriod(LPTMR0, TIMER_PERIOD);

		/* Enable timer interrupt */
		LPTMR_EnableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);
		EnableIRQ(LPTMR0_IRQn);

		/* Set LLWU to wakeup on LPTMR interrupt */
		LLWU_EnableInternalModuleInterruptWakup(LLWU, 0U, true);
		EnableIRQ(LLWU_IRQn);

		/* Start timer */
		LPTMR_StartTimer(LPTMR0);

		// Turn on red LED to indicate sleep
		GPIO_PortClear(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
		PRINTF("Setting alarm for ~%u s...\r\n", TIMER_PERIOD);

		// Set the power mode to VLLS0
		smc_power_mode_vlls_config_t smcVllsConfig = {kSMC_StopSub1, true};
		SMC_PreEnterStopModes();
		SMC_SetPowerModeVlls(SMC, &smcVllsConfig);
		// LLWU will wake up MCU from VLLS0 after LPTMR counts to defined value
		//SMC_PostExitStopModes();

//		GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
//		smc_power_state_t powerState =  SMC_GetPowerModeState(SMC);
//		PRINTF("Exited VLLS. Current power mode is: %u\r\n", powerState);
	}

    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}
