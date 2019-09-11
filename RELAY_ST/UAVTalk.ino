/**
 ******************************************************************************
 *
 *  ----- Based on the original code from MinOPOSD project -----
 *
 * @mods       by Julian Lilov
 *
 * @file       UAVTalk.ino
 * @author     Joerg-D. Rothfuchs
 * @brief      Implements a subset of the telemetry communication between
 *             OpenPilot CC, CC3D, Revolution and Ardupilot Mega MinimOSD
 *             with code from OpenPilot and MinimOSD.
 * @see        The GNU Public License (GPL) Version 3
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


#include "UAVTalk.h"

static unsigned long last_flighttelemetry_connect = 0;

// CRC lookup table
static const PROGMEM uint8_t crc_table[256] = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
	0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
	0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
	0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
	0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
	0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
	0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
	0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
	0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
	0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
	0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};



static inline int8_t uavtalk_get_int8(uavtalk_message_t *msg, int pos)
{
    return msg->Data[pos];
}


static inline int16_t uavtalk_get_int16(uavtalk_message_t *msg, int pos)
{
    int16_t i;

    memcpy(&i, msg->Data + pos, sizeof(int16_t));
    return i;
}


static inline int32_t uavtalk_get_int32(uavtalk_message_t *msg, int pos)
{
    int32_t i;

    memcpy(&i, msg->Data + pos, sizeof(int32_t));
    return i;
}


static inline float uavtalk_get_float(uavtalk_message_t *msg, int pos)
{
    float f;

    memcpy(&f, msg->Data + pos, sizeof(float));
    return f;
}


uint8_t uavtalk_parse_char(uint8_t c, uavtalk_message_t *msg)
{
    static uint8_t status = UAVTALK_PARSE_STATE_WAIT_SYNC;
    static uint8_t crc    = 0;
    static uint8_t cnt    = 0;
    static uint64_t ts 	  = 0;
    static uint8_t ts_len = 0;
    static bool timestamped_message = false;

    switch (status) {
    case UAVTALK_PARSE_STATE_WAIT_SYNC:
        if (c == UAVTALK_SYNC_VAL) {
            status    = UAVTALK_PARSE_STATE_GOT_SYNC;
            msg->Sync = c;
            crc = pgm_read_byte(&crc_table[0 ^ c]);
        }
        break;
    case UAVTALK_PARSE_STATE_GOT_SYNC:
        crc = pgm_read_byte(&crc_table[crc ^ c]);
        if ((c & UAVTALK_TYPE_MASK) == UAVTALK_TYPE_VER) {
            status = UAVTALK_PARSE_STATE_GOT_MSG_TYPE;

            timestamped_message = ((c & 0x80) != 0);
            if (timestamped_message) {
                ts_len = 4;
            }
            else {
                ts_len = 0;
            }

            msg->MsgType = c;
            cnt    = 0;
        } else {
            status = UAVTALK_PARSE_STATE_WAIT_SYNC;
        }
        break;
    case UAVTALK_PARSE_STATE_GOT_MSG_TYPE:
        crc = pgm_read_byte(&crc_table[crc ^ c]);
        cnt++;
        if (cnt < 2) {
            msg->Length = ((uint16_t)c);
        } else {
            msg->Length += ((uint16_t)c) << 8;
            if ((msg->Length < HEADER_LEN) || (msg->Length > 255 + HEADER_LEN)) {
                // Drop corrupted messages:
                // Minimal length is HEADER_LEN
                // Maximum is HEADER_LEN + 255 (Data) + 2 (Optional Instance Id)
                // As we are not parsing Instance Id, 255 is a hard maximum.
                status = UAVTALK_PARSE_STATE_WAIT_SYNC;
            } else {
                status = UAVTALK_PARSE_STATE_GOT_LENGTH;
                cnt    = 0;
            }
        }
        break;
    case UAVTALK_PARSE_STATE_GOT_LENGTH:
        crc = pgm_read_byte(&crc_table[crc ^ c]);
        cnt++;
        switch (cnt) {
        case 1:
            msg->ObjID  = ((uint32_t)c);
            break;
        case 2:
            msg->ObjID += ((uint32_t)c) << 8;
            break;
        case 3:
            msg->ObjID += ((uint32_t)c) << 16;
            break;
        case 4:
            msg->ObjID += ((uint32_t)c) << 24;
            status = UAVTALK_PARSE_STATE_GOT_OBJID;
            cnt    = 0;
            break;
        }
        break;
    case UAVTALK_PARSE_STATE_GOT_OBJID:
        crc = pgm_read_byte(&crc_table[crc ^ c]);
        cnt++;
        switch (cnt) {
        case 1:
            msg->InstID  = ((uint32_t)c);
            break;
        case 2:
            msg->InstID += ((uint32_t)c) << 8;
            if (msg->Length == HEADER_LEN) { // no data exists and no way to have timestamp info
                status = UAVTALK_PARSE_STATE_GOT_DATA;
            } else {
                status = UAVTALK_PARSE_STATE_GOT_INSTID;
            }
            cnt = 0;
            break;
        }
        break;
    case UAVTALK_PARSE_STATE_GOT_INSTID:
        if (timestamped_message) {
            crc = pgm_read_byte(&crc_table[crc ^ c]);
            cnt++;
            switch (cnt) {
                case 1:
                    ts  = ((uint32_t)c);
                    break;
                case 2:
                    ts += ((uint32_t)c) << 8;
                    break;
                case 3:
                    ts += ((uint32_t)c) << 16;
                    break;
                case 4:
                    ts += ((uint32_t)c) << 24;
                    timestamped_message = false;
                    cnt    = 0;
                    break;
            }
        }
        else {
            crc = pgm_read_byte(&crc_table[crc ^ c]);
            cnt++;
            msg->Data[cnt - 1] = c;
            if (cnt >= (msg->Length - HEADER_LEN - ts_len)) {
                status = UAVTALK_PARSE_STATE_GOT_DATA;
                cnt    = 0;
            }
        }
        break;
    case UAVTALK_PARSE_STATE_GOT_DATA:
        msg->Crc = c;
        status   = UAVTALK_PARSE_STATE_GOT_CRC;
        break;
    }

    if (status == UAVTALK_PARSE_STATE_GOT_CRC) {
        status = UAVTALK_PARSE_STATE_WAIT_SYNC;
        if (crc == msg->Crc) {
            return msg->Length;
        } else {
            return 0;
        }
    }
    else {
        return 0;
    }
}

int uavtalk_read(void)
{
    uavtalk_message_t msg;

    // grabbing data
    while (Serial.available() > 0) {
        uint8_t c = Serial.read();

        // parse data to msg
        uint16_t UAVOLength = uavtalk_parse_char(c, &msg);
        if (UAVOLength) {

          #ifdef OLEDLCD  
            telemetry_ok = true;
            lastpacketreceived = millis();
          #endif  

          #ifdef LOGGING
            if (logging_now) {
              unsigned long timestamp = millis() - logging_start_millis;
              logsize += dataFile.write((const uint8_t *)&timestamp, 4);

              uint64_t DataBlockSize = UAVOLength + 1;    // UAVOLength is Header + Data combined length, add +1 for checksum
              logsize += dataFile.write((const uint8_t *)&DataBlockSize, 8);

              logsize += dataFile.write((const uint8_t *)&msg.Sync, HEADER_LEN);
              logsize += dataFile.write((const uint8_t *)&msg.Data, UAVOLength-HEADER_LEN);
              logsize += dataFile.write((const uint8_t *)&msg.Crc, 1);
            }
          #endif  

            // consume msg
            switch (msg.ObjID) {

              case ATTITUDESTATE_OBJID:
                last_flighttelemetry_connect = millis();
                break;
            case GPSPOSITIONSENSOR_OBJID_003:
                osd_lat         = uavtalk_get_int32(&msg, GPSPOSITION_OBJ_LAT) / 10000000.0;
                osd_lon         = uavtalk_get_int32(&msg, GPSPOSITION_OBJ_LON) / 10000000.0;
                osd_satellites_visible = uavtalk_get_int8(&msg, GPSPOSITION_OBJ_SATELLITES);
                osd_fix_type    = uavtalk_get_int8(&msg, GPSPOSITION_OBJ_STATUS);
                osd_alt         = uavtalk_get_float(&msg, GPSPOSITION_OBJ_ALTITUDE);
                break;

            case GPSTIME_OBJID:
                osd_time_hour   = uavtalk_get_int8(&msg, GPSTIME_OBJ_HOUR);
                osd_time_minute = uavtalk_get_int8(&msg, GPSTIME_OBJ_MINUTE);
                osd_time_second = uavtalk_get_int8(&msg, GPSTIME_OBJ_SECOND);
                osd_time_day = uavtalk_get_int8(&msg, GPSTIME_OBJ_DAY);
                osd_time_month = uavtalk_get_int8(&msg, GPSTIME_OBJ_MONTH);
                osd_time_year = uavtalk_get_int16(&msg, GPSTIME_OBJ_YEAR);
              #ifdef OLEDLCD
                if (osd_time_year > 2016) {
                  valid_timestamp = true;
                }
              #endif  
                break;


            case OPLINKSTATUS_OBJID_003:
                uav_rssi = uavtalk_get_int8(&msg, OPLINKSTATUS_OBJ_RSSI);
                uav_linkquality = uavtalk_get_int8(&msg, OPLINKSTATUS_OBJ_LINKQUALITY);
                if ((uav_linkquality == 0) || (uav_rssi <= -127) || (uav_rssi == 0)) {
                    rssi_ok = false;
                }
                 else {
                    rssi_ok = true;   
                 }
                break;

            }
        }
    }

    // check connect timeout
    if (last_flighttelemetry_connect + FLIGHTTELEMETRYSTATS_CONNECT_TIMEOUT < millis()) {

    #ifdef OLEDLCD  
      telemetry_ok = false;
      valid_timestamp = false;
    #endif
    
    }

    return 1;
}

