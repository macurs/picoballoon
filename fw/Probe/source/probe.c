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
 * @file    probe.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "K32L2B21A.h"
#include "fsl_debug_console.h"
#include "fsl_smc.h"
#include "fsl_llwu.h"
#include "fsl_adc16.h"
#include "fsl_lptmr.h"
#include "fsl_tpm.h"
#include "fsl_rcm.h"
#include "fsl_pmc.h"
#include "fsl_spi.h"
#include "fsl_lpuart.h"

#include "gps.h"
#include "sf.h"
#include "sf_ol23xx.h"

/* definitions and declarations */
#define TIMER_PERIOD (60U * 15U) // length of sleep ( minutes * seconds )
//#define TIMER_PERIOD (10U) // length of sleep ( minutes * seconds )
#define ADC16_BASE          ADC0
#define ADC16_CHANNEL_GROUP 0U
/* Battery voltage pin is channel AD4B, need to set channel to 4 and mux to B */
#define ADC16_USER_CHANNEL  4U
#define ADC16_CHANNEL_MUX   kADC16_ChannelMuxB
/* low power modes */
#define USE_LPWR_MODES 0U
/* MS5611 temperature and pressure sensor */
#define MS5611_SPI_INSTANCE        SPI1
/* Length of SPI commands to the MS5611 sensor (in bytes) */
#define MS5611_CMDLEN              1U
/* Length of response to SPI commands from the MS5611 sensor (in bytes) */
#define MS5611_RCVLEN_RST          0U
#define MS5611_RCVLEN_RD_PROM      2U
#define MS5611_RCVLEN_CONV         0U
#define MS5611_RCVLEN_RD_ADC       3U
/* MS5611 commands */
#define MS5611_CMD_RST             0x1E
#define MS5611_CMD_RD_PROM         0xA0
#define MS5611_CMD_CONV_TEMP_BASE  0x50
#define MS5611_CMD_CONV_PRES_BASE  0x40
#define MS5611_CMD_RD_ADC          0x00
/* Oversampling rates for ADC conversion - command offset */
#define MS5611_OSR_256             0U
#define MS5611_OSR_512             2U
#define MS5611_OSR_1024            4U
#define MS5611_OSR_2048            6U
#define MS5611_OSR_4096            8U
/* Select oversampling rates ofr temperature and pressure convert commands */
#define MS5611_OSR_TEMP            MS5611_OSR_1024
#define MS5611_OSR_PRES            MS5611_OSR_1024

#define MS5611_CMD_CONV_TEMP      (MS5611_OSR_TEMP + MS5611_CMD_CONV_TEMP_BASE)
#define MS5611_CMD_CONV_PRES      (MS5611_OSR_PRES + MS5611_CMD_CONV_PRES_BASE)
/* buffer size for sending and receiving data */
#define MS5611_BUFFER_SIZE 4U

/* globals */
bool g_uartRxIdleFlag = false;
uint8_t g_rxBuffer[RX_DATA_SIZE] = {0,};


/*!
 * @brief Gets source clock for SPI peripheral.
 */
#define GET_SPI_MODULE_CLK() \
    (CLOCK_GetFreq(kCLOCK_BusClk))

/*!
 * @brief Initializes SIGFOX device and SW driver.
 *
 * @param drvData Driver run-time data.
 *
 * @return Status result of the function.
 */
static status_t SetupSigfoxDriver(sf_drv_data_t *drvData)
{
    sf_user_config_t userConfig;

    SF_GetDefaultConfig(&userConfig);
    userConfig.xtal = sfXtal276;

    /* GPIOs initialization.
     * Note: GPIO settings are place in pin_mux.h file. */
    /* ACK pin. */
    drvData->gpioConfig.ackPin.gpioInstance = instanceC;
    drvData->gpioConfig.ackPin.gpioPinNumber = 20U;

    /* CS pin. */
    drvData->gpioConfig.csPin.gpioInstance = instanceC;
    drvData->gpioConfig.csPin.gpioPinNumber = 4U;

    SF_SetupGPIOs(&(drvData->gpioConfig));

    /* SPI initialization. */
    drvData->spiConfig.baudRate = 125000U;
    drvData->spiConfig.sourceClkHz = GET_SPI_MODULE_CLK();
    drvData->spiConfig.spiInstance = 0U;

    SF_SetupSPI(&(drvData->spiConfig), NULL);

    /* Sigfox driver initialization.
     * Note: drvData->gpioConfig and drvData->spiConfig structures are used
     * by SF_SetupGPIOs, SF_SetupSPI and SF_Init. */
    return SF_Init(drvData, &userConfig);
}

/*!
 * @brief Fill Tx buffer with MS5611 command and pad with 1s
 *
 * @param xfer SPI transfer structure to be set up
 *
 * @param cmd MS5611 SPI command
 *
 * @param len MS5611 SPI response length to selected command (in bytes)
 */
void SendSensorCmd(spi_transfer_t *xfer, uint8_t *txBuffer, uint8_t *rxBuffer, uint8_t cmd, uint8_t len)
{
	/* Prepare SPI transfer structure */
	xfer->txData = txBuffer;
	xfer->rxData = rxBuffer;
	xfer->txData[0] = cmd;
	xfer->dataSize = len + MS5611_CMDLEN;
	/* Fill rest of tx buffer after command with dummy data, to read response from MS5611 */
	for (int i = 1; i < xfer->dataSize; i++)
	{
		xfer->txData[i] = 0xFF;
	}
	/* CS low */
	GPIO_PinWrite(BOARD_INITPINS_SENS_CS_GPIO, BOARD_INITPINS_SENS_CS_PIN, 0U);
	/* Send command and read response */
	SPI_MasterTransferBlocking(MS5611_SPI_INSTANCE, xfer);
	/* CS high */
	GPIO_PinWrite(BOARD_INITPINS_SENS_CS_GPIO, BOARD_INITPINS_SENS_CS_PIN, 1U);
}

/*!
 * @brief Initialize ADC peripheral.
 *
 * @param adcBase Base address of ADC peripheral to be initialized.
 *
 * @param adc16ChannelConfigStruct Structure to be used with ADC16_SetChannelConfig.
 */

status_t SetupADC(ADC_Type* base, adc16_channel_config_t* adc16ChannelConfigStruct)
{
	status_t status = kStatus_Success;
	adc16_config_t adc16ConfigStruct;
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

	/* enable low power */
	adc16ConfigStruct.enableLowPower = true;

	ADC16_Init(base, &adc16ConfigStruct);

	ADC16_SetChannelMuxMode(base, ADC16_CHANNEL_MUX);

	ADC16_EnableHardwareTrigger(base, false); /* Make sure the software trigger is used. */
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
	status = ADC16_DoAutoCalibration(base);
	if (kStatus_Success == status)
	{
		PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
	}
	else
	{
		PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
	}
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

	adc16ChannelConfigStruct->channelNumber                        = ADC16_USER_CHANNEL;
	adc16ChannelConfigStruct->enableInterruptOnConversionCompleted = false;
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
	adc16ChannelConfigStruct->enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

	return status;
}


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

/* LPUART user callback */
void LPUART_UserCallback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{

    if (kStatus_LPUART_RxIdle == status || kStatus_LPUART_RxHardwareOverrun == status || kStatus_LPUART_RxRingBufferOverrun == status)
    {
    	g_uartRxIdleFlag = true;
    }
}

/*
 * @brief calculate the CRC code for ms5611
 *
 * @param n_prom sensor prom array
 *
 * @return crc code
*/
unsigned char CalculateSensorCrc4(unsigned int n_prom[])
{
	int cnt; // simple counter
	unsigned int n_rem; // crc reminder
	unsigned int crc_read; // original value of the crc
	unsigned char n_bit;
	n_rem = 0x00;
	crc_read=n_prom[7]; //save read CRC
	n_prom[7]=(0xFF00 & (n_prom[7])); //CRC byte is replaced by 0
	for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
	{// choose LSB or MSB
		if (cnt%2==1)
		{
			n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
		}
		else
		{
			n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
		}
		for (n_bit = 8; n_bit > 0; n_bit--)
		{
			if (n_rem & (0x8000))
			{
				n_rem = (n_rem << 1) ^ 0x3000;
			}
			else
			{
				n_rem = (n_rem << 1);
			}
		}
	}
	n_rem= (0x000F & (n_rem >> 12)); // final 4-bit reminder is CRC code
	n_prom[7]=crc_read; // restore the crc_read to its original place
	return (n_rem ^ 0x0);
}

/*
 * @brief   Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitBootPins();

	/* Handle wakeup from VLLS. */
	if (kRCM_SourceWakeup & RCM_GetPreviousResetSources(RCM))
	{
		PMC_ClearPeriphIOIsolationFlag(PMC);
		NVIC_ClearPendingIRQ(LLWU_IRQn);
	}

    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    /*
     * TODO: handle debug console when entering/leaving low power modes
     */

    /* Power modes VLPR and VLLS need to be explicitly enabled */
	SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
#if defined(USE_LPWR_MODES) && USE_LPWR_MODES
	status_t powerStatus;
	powerStatus = SMC_SetPowerModeVlpr(SMC);
	if (kStatus_Success != powerStatus)
	{
		PRINTF("Error entering VLPR mode: %d\r\n");
	} else {
		PRINTF("Successfully entered VLPR mode\r\n");
	}
#endif

	/*
	 * Battery voltage read using ADC module
	 */

	/* Setup */
	adc16_channel_config_t adc16ChannelConfigStruct;
	SetupADC(ADC16_BASE, &adc16ChannelConfigStruct);

	/*
	 When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
	 function, which works like writing a conversion command and executing it. For another channel's conversion,
	 just to change the "channelNumber" field in channel's configuration structure, and call the
	 "ADC16_ChannelConfigure() again.
	*/
	ADC16_SetChannelConfig(ADC16_BASE, ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
	while (0U == (kADC16_ChannelConversionDoneFlag &
				  ADC16_GetChannelStatusFlags(ADC16_BASE, ADC16_CHANNEL_GROUP)))
	{
	}

	/* Start conversion with SW trigger */
	uint32_t adcValue = ADC16_GetChannelConversionValue(ADC16_BASE, ADC16_CHANNEL_GROUP);

	/* convert voltage from 12 bits to 8 bits */
	uint8_t convertedVoltage = adcValue >> 4;

	PRINTF("ADC read voltage: %d; msg payload: 0x%X\r\n", adcValue, convertedVoltage);

	/*
	 * MS5611 temperature and pressure sensor
	 */

	/* SPI1 instance is initalized in board functions */
	uint8_t txBuffer[MS5611_BUFFER_SIZE];
	uint8_t rxBuffer[MS5611_BUFFER_SIZE];
	spi_transfer_t xfer;
	/* Reset MS5611 Sensor */
	SendSensorCmd(&xfer, txBuffer, rxBuffer, MS5611_CMD_RST, MS5611_RCVLEN_RST);
	/* Wait 2.8 ms after reset */
	for (uint16_t i = 0; i < 40000; i++)
	{
		__asm volatile("nop");
	}
	/* Read calibration data */
	uint32_t sensProm[8];
	for (uint8_t i = 0; i < 8; i++)
	{
		SendSensorCmd(&xfer, txBuffer, rxBuffer, MS5611_CMD_RD_PROM + (i << 1), MS5611_RCVLEN_RD_PROM);
		/* MSB first */
		sensProm[i] = rxBuffer[MS5611_CMDLEN] << 8 | rxBuffer[MS5611_CMDLEN + 1];
//		PRINTF("Sensor calibration data word #%u: %u (%x)\r\n", i, sensProm[i], sensProm[i]);
	}

	if (CalculateSensorCrc4(sensProm) == (sensProm[7] & 0x0F)){
		PRINTF("MS5611 sensor calibration data CRC check success.\r\n");
	} else {
		PRINTF("MS5611 sensor calibration data CRC check fail.\r\n");
	}

	/* Convert pressure command */
	SendSensorCmd(&xfer, txBuffer, rxBuffer, MS5611_CMD_CONV_PRES, MS5611_RCVLEN_CONV);

	/* Wait to finish conversion */
	for (uint16_t i = 0; i < 65535; i++)
	{
		__asm volatile("nop");
	}

	/* Read pressure command */
	SendSensorCmd(&xfer, txBuffer, rxBuffer, MS5611_CMD_RD_ADC, MS5611_RCVLEN_RD_ADC);
	uint32_t D1 = rxBuffer[MS5611_CMDLEN] << 16 | rxBuffer[MS5611_CMDLEN + 1] << 8 | rxBuffer[MS5611_CMDLEN + 2];

	/* Convert temperature command */
	SendSensorCmd(&xfer, txBuffer, rxBuffer, MS5611_CMD_CONV_TEMP, MS5611_RCVLEN_CONV);

	/* Wait to finish conversion */
	for (uint16_t i = 0; i < 65535; i++)
	{
		__asm volatile("nop");
	}

	/* Read temperature command */
	SendSensorCmd(&xfer, txBuffer, rxBuffer, MS5611_CMD_RD_ADC, MS5611_RCVLEN_RD_ADC);
	uint32_t D2 = rxBuffer[MS5611_CMDLEN] << 16 | rxBuffer[MS5611_CMDLEN + 1] << 8 | rxBuffer[MS5611_CMDLEN + 2];

	/* Calculate temperatue (MS5611 1st order algorithm) */
	int32_t   dT = D2 - (sensProm[5] << 8);
	int32_t    T = 2000 + (((int64_t) dT * (int32_t) sensProm[6]) >> 23);

	/* Calculate variables for pressure calculation */
	int64_t  OFF = (sensProm[2] << 16) + (((int64_t) dT * sensProm[4]) >> 7);
	int64_t SENS = (sensProm[1] << 15) + (((int64_t) dT * sensProm[3]) >> 8);

	/* Temperature and pressure accuracy improvement (MS5611 2nd order temperature compensation) */
	int32_t T2    = 0;
	int32_t OFF2  = 0;
	int32_t SENS2 = 0;
	if (T < 2000)
	{
		T2    = (int64_t) dT * dT >> 31;
		OFF2  = 5 * (T - 2000) * (T - 2000) >> 1;
		SENS2 = OFF2 >> 2; /* SENS2 = 5 * (T - 2000) * (T - 2000) >> 2; */
	}
	if (T < -1500)
	{
		OFF2  += 7 * (T + 1500) * (T + 1500);
		SENS2 += 11 * (T + 1500) * (T + 1500) >> 1;
	}
	T    -= T2;
	OFF  -= OFF2;
	SENS -= SENS2;

	/* Calculate temperature compensated pressure */
	int32_t    P = ((D1 * SENS >> 21) - OFF) >> 15;

	PRINTF("Temperature: %d.%d°C; Atmospheric pressure: %d.%dhPa\r\n", T/100, T%100, P/100, P%100);

	/*
	 * GPS
	 */

	gps_info_t gps_info;

	lpuart_handle_t lpuartHandle;
	lpuart_transfer_t receiveXfer;
	receiveXfer.rxData   = g_rxBuffer;
	receiveXfer.dataSize = RX_DATA_SIZE;
	size_t receivedBytes;
	status_t lpuartStatus;
	uint8_t rxRingBuffer[RX_RING_BUFFER_SIZE] = {0,}; /* RX ring buffer. */

	/* See BOARD_... functions for LPUART0 init */

	/* Setup async UART communication*/
	LPUART_TransferCreateHandle(GNSS_LPUART_INSTANCE, &lpuartHandle, LPUART_UserCallback, NULL);

	LPUART_ClearStatusFlags(GNSS_LPUART_INSTANCE, 0xFFFFFFFF);
	/* Enable interrupts: idle rx line, newline character */
	LPUART_EnableInterrupts(GNSS_LPUART_INSTANCE, kLPUART_LinBreakInterruptEnable | kLPUART_TransmissionCompleteInterruptEnable | kLPUART_RxDataRegFullInterruptEnable);
	LPUART_DisableInterrupts(GNSS_LPUART_INSTANCE, kLPUART_RxOverrunInterruptEnable);

	LPUART_TransferStartRingBuffer(GNSS_LPUART_INSTANCE, &lpuartHandle, rxRingBuffer, RX_RING_BUFFER_SIZE);
//    lpuartRxStatus = LPUART_TransferReceiveNonBlocking(GNSS_LPUART_INSTANCE, &lpuartHandle, &receiveXfer, &receivedBytes);

	/* Wakeup from using FORCE_ON pin */
	GPIO_PinWrite(BOARD_INITPINS_GNSS_WU_GPIO, BOARD_INITPINS_GNSS_WU_PIN, 1U);
	for(int i = 0; i < 2400000; i++);
	GPIO_PinWrite(BOARD_INITPINS_GNSS_WU_GPIO, BOARD_INITPINS_GNSS_WU_PIN, 0U);
	/* Add delay after wakeup */
	for(int i = 0; i < 600000; i++);

	/* Check 3D_FIX pin status */
	uint8_t j = 64;
	bool gps_fix = false;
	gps_fix = GPIO_PinRead(BOARD_INITPINS_GNSS_FIX_GPIO, BOARD_INITPINS_GNSS_FIX_PIN);
	PRINTF("3D_FIX: %u\r\n", gps_fix);
	while (!gps_fix && j--)
	{
		for (int i = 0; i < 1000000; i++);
		gps_fix = GPIO_PinRead(BOARD_INITPINS_GNSS_FIX_GPIO, BOARD_INITPINS_GNSS_FIX_PIN);
		PRINTF("3D_FIX: %u\r\n", gps_fix);
	}

	uint32_t lat = 0;
	uint32_t lng = 0;
	uint16_t alt = 0;

	j = 6;
	while (j--){
		g_uartRxIdleFlag = false;
		while(!g_uartRxIdleFlag){
			__WFI();
		}

		lpuartStatus = LPUART_TransferReceiveNonBlocking(GNSS_LPUART_INSTANCE, &lpuartHandle, &receiveXfer, &receivedBytes);
		if (lpuartStatus){
			PRINTF("LPUART RX status: %d\r\n", lpuartStatus);
			continue;
		}
		PRINTF("\r\nReceived from L96:\r\n%.32s... (%d)\r\n", g_rxBuffer, strlen((char*)g_rxBuffer));

		lpuartStatus = L96_GetGpsInfo(&gps_info, g_rxBuffer);

		PRINTF("\r\nData parse return: %u\r\nLat %u\r\nLng %u\r\nAlt %u\r\n\r\n", lpuartStatus, gps_info.lat, gps_info.lng, gps_info.alt);
		if (gps_info.lat != 0) {
			lat = gps_info.lat;
		}
		if (gps_info.lng != 0) {
			lng = gps_info.lng;
		}
		if (gps_info.alt != 0) {
			alt = gps_info.alt;
		}
	}

	/* Read latitude, longitude, altitude */

	/* Switch to backup mode */
	lpuart_transfer_t sendXfer;
	sendXfer.txData   = (uint8_t*) L96_SLEEP_CMD;
	sendXfer.dataSize = sizeof(L96_SLEEP_CMD);
	lpuartStatus = LPUART_TransferSendNonBlocking(GNSS_LPUART_INSTANCE, &lpuartHandle, &sendXfer);
	PRINTF("Sent sleep cmd: %s\r\n", lpuartStatus == kStatus_Success ? "success" : "fail");

	/*
	 * Sigfox module sends data
	 */

	/* construct message */
	uint8_t message[12] = {0};
	message[0] = convertedVoltage;                    /* Voltage     1 Byte */
	message[1] = (T + 4000) / 50;                     /* Temperature 1 Byte */
	message[2] = (uint8_t) ((0xFF00 & (P / 2)) >> 8); /* Pressure    2 Bytes */
	message[3] = (uint8_t) (0xFF & (P / 2));
	message[4] = (uint8_t) ((0xFF00 & alt) >> 8); /* Altitude    2 Bytes */
	message[5] = (uint8_t) (0xFF & alt);
	message[6] = (uint8_t) ((0xFF0000 & lat) >> 16); /* Latitude    3 Bytes */
	message[7] = (uint8_t) ((0xFF00 & lat) >> 8);
	message[8] = (uint8_t) (0xFF & lat);
	message[9] = (uint8_t) ((0xFF0000 & lng) >> 16); /* Longitude   3 Bytes*/
	message[10] = (uint8_t) ((0xFF00 & lng) >> 8);
	message[11] = (uint8_t) (0xFF & lng);

	/* Reset Sigfox module using ~SF_RESET GPIO pin */
	GPIO_PinWrite(BOARD_INITPINS_SF_RESET_GPIO, BOARD_INITPINS_SF_RESET_PIN, 0U);
	for(int i=0;i<100000;i++);
	GPIO_PinWrite(BOARD_INITPINS_SF_RESET_GPIO, BOARD_INITPINS_SF_RESET_PIN, 1U);
	for(int i=0;i<100000;i++);

	/* Init Sigfox driver */
	sf_drv_data_t sfDrvData;
	status_t sfResult;
	sfResult = SetupSigfoxDriver(&sfDrvData);
	if (sfResult != kStatus_Success)
	{
		PRINTF("An error occurred in SetupSigfoxDriver (%d)\r\n", sfResult);
	}

	/* Get Sigfox device info */
//	sf_device_info_t devInfo = {0U,};
//	sfResult = SF_GetDeviceInfo(&sfDrvData, &devInfo);

	/* Send Sigfox message */
	sf_msg_payload_t sfPayload;
	sfPayload.payloadLen = 12U;
	sfPayload.payload = message;
	PRINTF("Transmitting Sigfox message...\n");
	sfResult = SF_SendPayload(&sfDrvData, &sfPayload);
//		sfResult = sfStatusSuccess;
	if (kStatus_Success == sfResult) {
		PRINTF("Sigfox message sent successfully.\r\n");
	} else {
		PRINTF("Something went wrong... sf_status %u\r\n", sfResult);
	}

	/* Send Sigfox module to sleep to save power */
	sfResult = SF_Sleep(&sfDrvData);

	/* LPTMR **********************/

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

	PRINTF("Setting alarm for ~%u s...\r\n", TIMER_PERIOD);

	/* Set the power mode to VLLS1 */
	smc_power_mode_vlls_config_t smcVllsConfig = {kSMC_StopSub1, true};
	SMC_PreEnterStopModes();
	SMC_SetPowerModeVlls(SMC, &smcVllsConfig);

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
