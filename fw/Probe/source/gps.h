/*
 * gps.h
 *
 *  Created on: 9. 5. 2023
 *      Author: macur
 */

#ifndef GPS_H_
#define GPS_H_

#include <stdbool.h>
#include <stdint.h>
#include "fsl_common.h"

#define GNSS_LPUART_INSTANCE LPUART0
#define RX_RING_BUFFER_SIZE 512U
#define RX_DATA_SIZE 500U
#define L96_SLEEP_CMD       "$PMTK225,4*2F\r\n"
#define L96_MODE_BALLON_CMD "$PMTK886,3*2B\r\n"
#define L96_MODE_NORMAL_CMD "$PMTK886,0*28\r\n"
#define L96_SET_OUTPUT_CMD  "$PMTK314,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35\r\n" /* Output only GLL, RMC, GGA messages */
/* balloon mode */
#define L96_PARSE_RETRY_TIMES 32U
#define NMEA_BUF_LEN 32U /* Maximum length of a single NMEA data field (between ',' characters) */

extern bool g_uartRxIdleFlag;
extern uint8_t g_rxBuffer[RX_DATA_SIZE];

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

status_t L96_GetGpsInfo(gps_info_t *gps_info, uint8_t *buffer);

#endif /* GPS_H_ */
