/**
 ******************************************************************************
 *
 *  ----- Based on the original code from MinOPOSD project -----
 *
 * @mods       by Julian Lilov to match the respective Librepilot 16.09
 *             UAVObjects definitions
 *
 * @file       UAVTalk.h
 * @author     Joerg-D. Rothfuchs
 * @brief      Implements a subset of the telemetry communication between
 *             OpenPilot CC, CC3D, Revolution and Ardupilot Mega MinimOSD
 *             with code from OpenPilot and MinimOSD.
 * @see        The GNU Public License (GPL) Version 3
 *
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/> or write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


#ifndef UAVTALK_H_
#define UAVTALK_H_


// TODO enhancement:
// Generate the following automatically out of the XML files.
//
// These object constants are version dependent!
//
// Short hints getting them manually:
// http://wiki.openpilot.org/display/Doc/Windows%3A+Building+and+Packaging
// git clone git://git.openpilot.org/OpenPilot.git OpenPilot
// QT Creator > Datei oder Projekt oeffnen... uavobjgenerator
// generate release and copy exe to <top>\ground\uavobjgenerator
// uavobjgenerator -flight ..\..\shared\uavobjectdefinition ..\..\
// //

#define ATTITUDESTATE_OBJID                 0xD7E0D964          // 16.09

#define GPSPOSITIONSENSOR_OBJID_003    	    0x9DF1F67A          // 16.09

#define GPSPOSITION_OBJ_LAT                 0
#define GPSPOSITION_OBJ_LON                 4
#define GPSPOSITION_OBJ_ALTITUDE            8
//#define GPSPOSITION_OBJ_GEOIDSEPARATION     12
//#define GPSPOSITION_OBJ_HEADING             16
//#define GPSPOSITION_OBJ_GROUNDSPEED         20
//#define GPSPOSITION_OBJ_PDOP                24
//#define GPSPOSITION_OBJ_HDOP                28
#//define GPSPOSITION_OBJ_VDOP                32
#define GPSPOSITION_OBJ_STATUS              36
#define GPSPOSITION_OBJ_SATELLITES          37


#define GPSTIME_OBJID                       0xD4478084			// 16.09
#define GPSTIME_OBJ_YEAR                    0
#define GPSTIME_OBJ_MONTH                   2
#define GPSTIME_OBJ_DAY                     3
#define GPSTIME_OBJ_HOUR                    4
#define GPSTIME_OBJ_MINUTE                  5
#define GPSTIME_OBJ_SECOND                  6


#define OPLINKSTATUS_OBJID_003         	    0xDED43774          // 16.09
#define OPLINKSTATUS_OBJ_RSSI          	    102	 	        	// 16.09
#define OPLINKSTATUS_OBJ_LINKQUALITY   	    26	       	        // 16.09



#define UAVTALK_MODE_PASSIVE                 0x01               // do not send any UAVTalk packets

#define FLIGHTTELEMETRYSTATS_CONNECT_TIMEOUT 10000
#define GCSTELEMETRYSTATS_SEND_PERIOD        1000

#if defined VERSION_RELEASE_12_10_1 || defined VERSION_RELEASE_12_10_2 || defined VERSION_RELEASE_13_06_1 || defined VERSION_RELEASE_13_06_2
#define HEADER_LEN                           8
#else
#define HEADER_LEN                           10
#endif

#define RESPOND_OBJ_LEN                      HEADER_LEN
#define REQUEST_OBJ_LEN                      HEADER_LEN

#define UAVTALK_SYNC_VAL                     0x3C

#define UAVTALK_TYPE_MASK                    0x70
#define UAVTALK_TYPE_VER                     0x20

#define UAVTALK_TYPE_OBJ                     (UAVTALK_TYPE_VER | 0x00)
#define UAVTALK_TYPE_OBJ_REQ                 (UAVTALK_TYPE_VER | 0x01)
#define UAVTALK_TYPE_OBJ_ACK                 (UAVTALK_TYPE_VER | 0x02)
#define UAVTALK_TYPE_ACK                     (UAVTALK_TYPE_VER | 0x03)
#define UAVTALK_TYPE_NACK                    (UAVTALK_TYPE_VER | 0x04)


typedef enum {
    UAVTALK_PARSE_STATE_WAIT_SYNC = 0,
    UAVTALK_PARSE_STATE_GOT_SYNC,
    UAVTALK_PARSE_STATE_GOT_MSG_TYPE,
    UAVTALK_PARSE_STATE_GOT_LENGTH,
    UAVTALK_PARSE_STATE_GOT_OBJID,
    UAVTALK_PARSE_STATE_GOT_INSTID,
    UAVTALK_PARSE_STATE_GOT_DATA,
    UAVTALK_PARSE_STATE_GOT_CRC
} uavtalk_parse_state_t;


typedef enum {
    TELEMETRYSTATS_STATE_DISCONNECTED = 0,
    TELEMETRYSTATS_STATE_HANDSHAKEREQ,
    TELEMETRYSTATS_STATE_HANDSHAKEACK,
    TELEMETRYSTATS_STATE_CONNECTED
} telemetrystats_state_t;


typedef struct __uavtalk_message {
    uint8_t  Sync;
    uint8_t  MsgType;
    uint16_t Length;
    uint32_t ObjID;
    uint16_t InstID;
    uint8_t  Data[255];
    uint8_t  Crc;
} uavtalk_message_t;


int uavtalk_read(void);
int uavtalk_state(void);


#endif /* UAVTALK_H_ */
