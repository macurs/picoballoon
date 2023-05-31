/*
 * gps.c
 *
 *  Created on: 26. 4. 2023
 *      Author: macur
 */

#include "gps.h"

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
