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
 * @file    L96_Setup.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "K32L2B21A.h"
#include "fsl_debug_console.h"

#include "fsl_lpuart.h"
#include "fsl_gpio.h"

#define GNSS_LPUART_INSTANCE LPUART0
#define RX_RING_BUFFER_SIZE 512U
#define RX_DATA_SIZE 500U
#define L96_SLEEP_CMD       "$PMTK225,4*2F\r\n"
#define L96_MODE_BALLON_CMD "$PMTK886,3*2B\r\n"
#define L96_MODE_NORMAL_CMD "$PMTK886,0*28\r\n"
#define L96_SET_OUTPUT_CMD  "$PMTK314,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35\r\n" /* Output only GLL, RMC, GGA messages */

#define L96_PARSE_RETRY_TIMES 32U
#define NMEA_BUF_LEN 32U /* Maximum length of a single NMEA data field (between ',' characters) */

bool g_uartRxIdleFlag = false;
uint8_t g_rxBuffer[RX_DATA_SIZE] = {0,};

/*
 * NMEA message parsing return types
 */
enum {
	gpsSuccess      =  0U, /* Successfully read position data from NMEA message */
	gpsIncomplete   =  1U, /* The NMEA message was not complete */
	gpsChecksum     =  2U, /* The NMEA message checksum did not match the calculated checksum */
	gpsDataNotFound =  4U, /* The position data was not in the NMEA message */
	gpsAck          =  8U, /* PMTK ACK message received */
	gpsError        = 16U  /* Generic error code */
};

/*
 * Position of parser in NMEA message
 */
enum {
    msgBefore    = 0U,
	msgIn        = 1U,
	msgAfter     = 2U
};

/*
 * Type of message
 */
enum {
	typeNmeaUnknown  = 0U,
	typeNmeaGga      = 1U,
	typeNmeaOther    = 2U,
	typePmtkAck      = 3U
};

typedef uint8_t nmea_ret_t;

struct _gps_info {
	uint32_t lat;
	uint32_t lng;
	uint16_t alt;
};

typedef struct _gps_info gps_info_t;

status_t L96_GetGpsInfo(gps_info_t *gps_info, uint8_t *buffer)
{
	uint32_t coord;
	uint32_t min;
	uint8_t deg;

	nmea_ret_t  status      = gpsError;
	uint16_t    charsRead   = 0U;
	uint8_t*    ringBufTail = buffer;
	uint8_t     str[NMEA_BUF_LEN] = {0U};
	uint8_t     strIdx      = 0;
	uint8_t     c;
	uint8_t     checksum;
	uint8_t     msgFieldIdx;
	uint8_t     retry       = L96_PARSE_RETRY_TIMES;
	uint8_t     nmeaMsgPos  = msgBefore;
	uint8_t     msgType     = typeNmeaUnknown;

	assert(buffer != NULL);

	/* Loop until GPS fix data is read from a message or until retry times run out */
	while(status && retry--)
	{
		if (status == gpsAck)
		{
			break;
		}

		if (gps_info != NULL)
		{
			gps_info->lat = gps_info->lng = 0U;
			gps_info->alt = 0U;
		}
		checksum    = 0U;           /* Clear checksum */
		msgFieldIdx = 0U;
		strIdx      = 0U;
		status      = gpsIncomplete; /* Return incomplete message status if ring buffer runs out */
		msgType     = typeNmeaUnknown;

		/* Loop at most through the whole length of the ring buffer */
		for(; charsRead < RX_DATA_SIZE; charsRead++)
		{
			/* Check for fields too long for the buffer */
			if (strIdx >= NMEA_BUF_LEN - 1)
			{
				status = gpsIncomplete;
				break;
			}
//
//			/* Check if ring buffer tail is outside ring buffer memory */
//			if(ringBufTail >= handle->rxRingBufferSize + handle->rxRingBuffer)
//			{
//				/* Wrap back to ring buffer start */
//				ringBufTail -= handle->rxRingBufferSize;
//			}

			c = *ringBufTail++;

			/*
			 * Check for the start of an NMEA message, which has the following format:
			 * $<TalkerID:2><SentenceFormatter:3>[,<Data>]*<Checksum:2>\r\n
			 */
			if (c == '$'){
				if(nmeaMsgPos >= msgIn) /* Unexpected $ character in a message */
				{
					break;     /* Clear checksum, start to parse new message from here */
				}
				nmeaMsgPos = msgIn;
				continue;
			}

			/* Read characters until start of message '$' is reached */
			if (nmeaMsgPos < msgIn)
			{
				continue;
			}

			/* Detect end of message */
			if (c == '*')
			{
				if (nmeaMsgPos >= msgAfter) /* Unexpected '*' character after end of message */
				{
					break;    /* Return checksum error */
				}
				strIdx = 0;
				nmeaMsgPos = msgAfter;
				continue;
			}

			/* Parse checksum and check end of message for correct format */
			if (nmeaMsgPos == msgAfter)
			{
				str[strIdx++] = c;
				if (strIdx == 4)
				{
					str[strIdx] = '\0';
					/* Check for correct message ending */
					if (str[2] != '\r' || str[3] != '\n')
					{
						nmeaMsgPos = msgBefore;
						break;
					}

					/* Compare calculated checksum to message checksum */
					if (((uint8_t)strtol((char*)str, NULL, 16) & 0xFF) == checksum)
					{
						status = gpsSuccess;
					} else {
						status = gpsChecksum;
					}

					if (msgType == typePmtkAck){
						status |= gpsAck;
					} else if (gps_info != NULL) {
						status |= (gps_info->lat != 0U && gps_info->lng != 0U && gps_info->alt != 0U) ? gpsSuccess : gpsDataNotFound;
					} else {
						status = gpsError;
					}
					nmeaMsgPos = msgBefore;
					break;
				}
				continue;
			}

			/*
			 * Buffer is in the body of an NMEA message.
			 * true == inNmeaBody
			 * Add the current character to the message checksum
			 */
			checksum ^= c; /* Checksum is XOR of all message characters between $ and * (non-inclusive) */

			/* Parse message fields */
			if (c == ',')
			{
				/* The end of a field is reached. Data may be parsed */
				str[strIdx] = '\0';
				msgFieldIdx++;
				strIdx = 0;
			} else {
				/* Inside field, read character and go to next iteration */
				str[strIdx++] = c;
				continue;
			}

			if (msgFieldIdx == 1U && msgType == typeNmeaUnknown)
			{
				if (str[2] == 'G' && str[3] == 'G' && str[4] == 'A')
				{
					msgType = typeNmeaGga;
				} else if (str[0] == 'P' && str[1] == 'M' && str[2] == 'T' && str[3] == 'K' && str[4] == '0' && str[5] == '0' && str[6] == '1') {
					msgType = typePmtkAck;
				} else {
					msgType = typeNmeaOther;
				}
				continue;
			}

			/* If the message does not have the desired GGA type, parse until the end */
			if (msgType == typeNmeaOther)
			{
				continue;
			}

			/* GGA message: */
			if (msgFieldIdx == 3U)     /* Lat */
			{
				if (str[0] == '\0')
				{
					continue;
				}
				/* "ddmm.mmmm[m][m]" */
				deg  =  0U;
				deg += (str[0] - '0') * 10U ;
				deg +=  str[1] - '0';

				min  =  0U;
				min += (str[2] - '0') * 100000U;
				min += (str[3] - '0') * 10000U;
				/* str[4] '.'*/
				min += (str[5] - '0') * 1000U;
				min += (str[6] - '0') * 100U;
				min += (str[7] - '0') * 10U;
				min +=  str[8] - '0';

				coord  = (uint64_t) 0x80000000 * deg / 90U;
				coord += (uint64_t) 0x80000000 * min / (10000U * 60U * 90U);

			}

			if (msgFieldIdx == 4U)     /* Latitude sign N/S */
			{
				if (str[0] == '\0')
				{
					continue;
				}
				if (gps_info != NULL)
				{
					if (str[0] == 'N')
					{
						gps_info->lat = 0x800000 + (coord >> 8);
					} else {
						gps_info->lat = 0x800000 - (coord >> 8);
					}
				}
			}

			if (msgFieldIdx == 5U)     /* Longitude */
			{
				if (str[0] == '\0')
				{
					continue;
				}
				/* "ddmm.mmmm[m][m]" */
				deg  =  0U;
				deg += (str[0] - '0') * 100U ;
				deg += (str[1] - '0') * 10U ;
				deg +=  str[2] - '0';

				min  =  0U;
				min += (str[3] - '0') * 100000U;
				min += (str[4] - '0') * 10000U;
				/* str[5] '.'*/
				min += (str[6] - '0') * 1000U;
				min += (str[7] - '0') * 100U;
				min += (str[8] - '0') * 10U;
				min +=  str[9] - '0';

				coord  = (uint64_t) 0x80000000 * deg / 180U;
				coord += (uint64_t) 0x80000000 * min / (10000U * 60U * 180U);

			}
			if (msgFieldIdx == 6U)     /* Longitude sign E/W */
			{
				if (str[0] == '\0')
				{
					continue;
				}
				if (gps_info != NULL)
				{
					if (str[0] == 'E')
					{
						gps_info->lng = 0x800000 + (coord >> 8);
					} else {
						gps_info->lng = 0x800000 - (coord >> 8);
					}
				}
			}
			if (msgFieldIdx == 10U)    /* Altitude */
			{
				uint8_t div = 0U;
				if (str[0] == '\0')
				{
					continue;
				}
				coord = 0U;
				while(str[strIdx] != '\0')
				{
					if (str[strIdx] == '.')
					{
						strIdx++;
						div = 1U;
						continue;
					}
					coord *= 10U;
					coord += str[strIdx++] - '0';
					div *= 10U;
				}
				if (div == 0U)
				{
					div = 1U;
				}
				if (gps_info != NULL)
				{
					gps_info->alt = coord * 5U / div;
				}
			}
		}
	}
	return status;
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
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

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
    uint8_t j = 32;
    bool gps_fix = false;
    gps_fix = GPIO_PinRead(BOARD_INITPINS_GNSS_FIX_GPIO, BOARD_INITPINS_GNSS_FIX_PIN);
	PRINTF("3D_FIX: %u\r\n", gps_fix);
    while (!gps_fix && j--)
    {
		for (int i = 0; i < 1000000; i++);
    	gps_fix = GPIO_PinRead(BOARD_INITPINS_GNSS_FIX_GPIO, BOARD_INITPINS_GNSS_FIX_PIN);
    	PRINTF("3D_FIX: %u\r\n", gps_fix);
    }



    uint32_t rxCount;
    j = 6;
    while (j--){
    	g_uartRxIdleFlag = false;
    	while(!g_uartRxIdleFlag){
//    		LPUART_TransferReceiveNonBlocking(GNSS_LPUART_INSTANCE, &lpuartHandle, &receiveXfer, &receivedBytes);
    		__WFI();
    	}


//    	LPUART_TransferStopRingBuffer(GNSS_LPUART_INSTANCE, &lpuartHandle);

    	lpuartStatus = LPUART_TransferGetReceiveCount(GNSS_LPUART_INSTANCE, &lpuartHandle, &rxCount);
    	if(!lpuartStatus)
    	{
			PRINTF("Rx count %u\r\n", rxCount);
    	}
    	else
    	{
    		PRINTF("Transfer status %u\r\n", lpuartStatus);
    	}
		PRINTF("Ring buffer length: %u\r\n", LPUART_TransferGetRxRingBufferLength(GNSS_LPUART_INSTANCE, &lpuartHandle));

		lpuartStatus = LPUART_TransferReceiveNonBlocking(GNSS_LPUART_INSTANCE, &lpuartHandle, &receiveXfer, &receivedBytes);
    	if (lpuartStatus){
    		PRINTF("LPUART RX status: %d\r\n", lpuartStatus);
    		continue;
    	}
    	PRINTF("\r\nReceived from L96:\r\n%.32s... (%d)\r\n", g_rxBuffer, strlen((char*)g_rxBuffer));

    	lpuartStatus = L96_GetGpsInfo(&gps_info, g_rxBuffer);

    	PRINTF("\r\nData parse return: %u\r\nLat %u\r\nLng %u\r\nAlt %u\r\n\r\n", lpuartStatus, gps_info.lat, gps_info.lng, gps_info.alt);

//    	LPUART_TransferStartRingBuffer(GNSS_LPUART_INSTANCE, &lpuartHandle, rxRingBuffer, RX_RING_BUFFER_SIZE);
    }

    /* Hot start command? */
    /* - may not be necessary - the module is restarted with the FORCE_ON pin and should perform a hot start automatically thanks to the data retained by RTC domain in backup mode */

    /* Setup UART to 115200 baud? */
    /* todo */

    /* Setup balloon mode / other */
    /* todo */

    /* Wait for GPS fix */
    /* todo: VLPS? - periodic check? */
//    while (!GPIO_PinRead(BOARD_INITPINS_GNSS_FIX_GPIO, BOARD_INITPINS_GNSS_FIX_PIN));


    /* Read latitude, longitude, altitude */

    /* Switch to backup mode */
    lpuart_transfer_t sendXfer;
	sendXfer.txData   = (uint8_t*) L96_SLEEP_CMD;
	sendXfer.dataSize = sizeof(L96_SLEEP_CMD);
    lpuartStatus = LPUART_TransferSendNonBlocking(GNSS_LPUART_INSTANCE, &lpuartHandle, &sendXfer);
    PRINTF("Sent sleep cmd: %s\r\n", lpuartStatus == kStatus_Success ? "success" : "fail");

    /* look for ack messgae after sleep command */
    j = 3;
    while (j--){
		for(int i = 0; i < 1200000; i++);
        lpuartStatus = LPUART_TransferReceiveNonBlocking(GNSS_LPUART_INSTANCE, &lpuartHandle, &receiveXfer, &receivedBytes);
    	if (lpuartStatus){
    		PRINTF("LPUART RX status: %d\r\n", lpuartStatus);
    	}
    	lpuartStatus = L96_GetGpsInfo(NULL, g_rxBuffer);
    	PRINTF("Sleep ack received? %s\r\n", lpuartStatus == gpsAck ? "yes" : "no" );
		if(lpuartStatus == gpsAck)
		{
			break;
		}
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
