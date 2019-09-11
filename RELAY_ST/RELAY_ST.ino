// RELAY STATION (Telemetry Display, Telemetry Logging on SD Card, WDT support (2 sec.), UAVTalk TimeStamp Fix
// ver 3.1.9

/**
 ******************************************************************************
 *
 * @file       RELAY_ST.ino
 * @author     Julian Lilov
 * @brief      Captures telemetry stream from Revolution to Oplink Mini,
 *             records it in OPL format on MicroSD card and displays info based
 *             on incoming GPS data like latitude, longitude, altutide, number
 *             of satelites, type of fix, GPS time, etc. on LCD display. Some
 *             code from the original MinOPOSD project is used.
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


// Upload tips:
// Select Arduino Pro Mini 16MGHz flashed with Optiboot, flash bootloader (set fuses!), then select Atmega328P Stand Alone (USBasp); Upload Using Programmer

// ** OLED I2C Display
// ** А4 - SDA
// ** A5 - SCL*/

// ** SD card attached to SPI bus as follows:
// ** MOSI - pin 11
// ** MISO - pin 12
// ** CLK - pin 13
// ** CS - pin 10 

// ** Buzzer - pin 9

// ** Voltage monitoring - A3


/* ************************************************************ */
/* ****************     CONFIGURATIONS     ******************** */
/* ************************************************************ */

#define OLEDLCD
#define LOGGING
#define STATION_BATTERY_MONITORING
#define USE_WDT
//#define DEBUG

#define VOLT_DIV_RATIO    15.56839615742696                   // Vref 1.1V based: RELAY-STATION Battery Monitor (may need adjustment)
#define MIN_VOLTAGE1 6.6f                                     // First battery alarm level. Will emit 2 short tones every 10 sec.
#define MIN_VOLTAGE2 6.0f                                     // Second battery alarm level. Will emit 1 short + 1 long tone every 5 sec

#define TELEMETRY_SPEED 57600                                 // How fast telemetry is coming to Serial port


/* ************************************************************ */
/* **************** MAIN PROGRAM - MODULES ******************** */
/* ************************************************************ */

#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data")))

#undef PSTR
#define PSTR(s) \
    (__extension__({ static prog_char __c[] PROGMEM = (s); &__c[0]; } \
                   ))

// AVR Includes
#include <FastSerial.h>
#include <Wire.h>

// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "wiring.h"
#endif


// Configurations
#include "UAVTalk.h"

#define REF_VOLTAGE       1.1                     // INTERNAL: a built-in reference, equal to 1.1 volts on the ATmega168 or ATmega328

#define CURRENT_VOLTAGE(x) ((x) * REF_VOLTAGE / 1024.0) * (VOLT_DIV_RATIO / 1.0)

#ifdef USE_WDT
#include <avr/wdt.h>
#endif

#ifdef LOGGING
#include <SD.h>
#endif

#ifdef OLEDLCD
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#define I2CADRESS 0x3c
#endif


/* *************************************************/
/* ***************** DEFINITIONS *******************/

#define BUZZER_PIN        9   //(PB1) Any PWM pin ((add a 100-150 ohm resistor between buzzer & ground)

uint8_t      buzzer_status = 0;    

#ifdef STATION_BATTERY_MONITORING
static float station_voltage = -1.0;
#endif

static uint8_t osd_satellites_visible = 0;     // number of satelites
static uint8_t osd_fix_type      = 0;          // GPS lock 0-1=no fix, 2=2D, 3=3D, 4=DGPS
static float osd_lat             = 0;          // latitude
static float osd_lon             = 0;          // longitude
static float osd_alt             = 0;          // altitude

static int16_t osd_time_year = 0;
static int8_t osd_time_month = 0;
static int8_t osd_time_day = 0;
static int8_t osd_time_hour = 0;
static int8_t osd_time_minute = 0;
static int8_t osd_time_second = 0;

static uint8_t osd_alt_cnt       = 0;          // counter for stable osd_alt
static float osd_alt_prev        = 0;          // previous altitude
static uint8_t osd_got_home      = 0;          // tells if got home position or not
static float osd_home_lat        = 0;          // home latitude
static float osd_home_lon        = 0;          // home longitude
static float osd_home_alt        = 0;          // home altitude
static float osd_home_distance   = 0;          // distance from home

uint8_t      uav_rssi = 0;                   // radio RSSI (%)
uint8_t      uav_linkquality = 0;            // radio link quality
boolean      rssi_ok = false;

#ifdef OLEDLCD

long lastpacketreceived;
static boolean telemetry_ok = false;
static boolean link_initiated = false;
static boolean valid_timestamp = false;

static uint8_t indicator = 1;

static unsigned long loop1hz_prevmillis; // 1hz loop

SSD1306AsciiWire display;

#endif

static unsigned long loop10hz_prevmillis; // 10hz loop

#ifdef LOGGING
static boolean SD_OK = false;
static boolean logging_now = false;
static unsigned long logging_start_millis;
static uint32_t logsize;
File dataFile;
#endif

// Objects and Serial definitions

FastSerialPort0(Serial);


/* **********************************************/
/* ***************** SETUP() *******************/

void setup()
{
#ifdef USE_WDT
  watchdogSetup();
#endif
  Serial.begin(TELEMETRY_SPEED);
  Serial.flush();
  
#ifdef DEBUG 
  char top;
  extern char *__brkval;
  extern char __bss_end;
  Serial.println( __brkval ? &top - __brkval : &top - &__bss_end);  
#endif  

#ifdef LOGGING
   pinMode(10, OUTPUT);
   SD_OK = SD.begin(10);
#endif

#ifdef STATION_BATTERY_MONITORING
  analogReference(INTERNAL); // INTERNAL: a built-in reference, equal to 1.1 volts on the ATmega168 or ATmega328
#endif    

#ifdef OLEDLCD

  Wire.begin();         

#ifdef USE_WDT
  wdt_reset();
#endif
  tone(BUZZER_PIN, 2500);    // Initial two beeps - module alive indication :)
  delay(50);
  noTone(BUZZER_PIN);
  delay(30);
  tone(BUZZER_PIN, 2500);
  delay(50);
  noTone(BUZZER_PIN);

  display.begin(&Adafruit128x64, I2CADRESS);
  display.set400kHz();  
  display.setFont(Adafruit5x7);  
  display.clear();  

  display.set2X();
  display.println(F("RELAY"));
  display.println(F("STATION"));
  display.set1X();
  display.println(F("ver 3.1.9"));
#ifdef USE_WDT
  wdt_reset();
#endif  

#ifdef LOGGING
  display.println(F(""));
  if (SD_OK)
  {
    display.println(F("SD Card OK"));
    for (int ii = 0; ii<3000; ii++)
    {
    #ifdef STATION_BATTERY_MONITORING
      updateVoltage();
    #endif  
    #ifdef USE_WDT
      wdt_reset();
    #endif  
      delay(1);
    }
  }
  else {
    display.println(F("SD Card FAULURE"));

    tone(BUZZER_PIN, 200); 
    delay(1000);
  #ifdef USE_WDT
    wdt_reset();
  #endif  
    delay(1000);
    noTone(BUZZER_PIN);
    for (int ii = 0; ii<3000; ii++)
    {
    #ifdef STATION_BATTERY_MONITORING
      updateVoltage();
    #endif  
    #ifdef USE_WDT
      wdt_reset();
    #endif  
      delay(1);
    }
  }
#else
  for (int ii = 0; ii<3000; ii++)
  {
  #ifdef STATION_BATTERY_MONITORING
    updateVoltage();
  #endif  
  #ifdef USE_WDT
    wdt_reset();
  #endif  
    delay(1);
  }
#endif

  display.clear();
  
  tone(BUZZER_PIN, 2500);
  delay(250);
  noTone(BUZZER_PIN);

  loop1hz_prevmillis = 0;
#endif  
  loop10hz_prevmillis = 0;

//  while (1);  - Enable this ONLY to test WDT behaviour

} // END of setup();


/* ***********************************************/
/* ***************** MAIN LOOP *******************/

void loop()
{
#ifdef USE_WDT
  wdt_reset();
#endif
  if ((loop10hz_prevmillis + 100) < millis() ) {
    loop10hz_prevmillis = millis();

    setHomeVars(); // calculate Distance from home
  
  #ifdef STATION_BATTERY_MONITORING
    updateVoltage();

    switch (buzzer_status) {
    case 1:
      playTones(1);
      break;
    case 2:
      playTones(2);
      break;
    default:
      break;            
    } 
  #endif    
    
  }

#ifdef OLEDLCD
  if ((loop1hz_prevmillis + 1000) < millis() ) {
    loop1hz_prevmillis = millis();

    refresh_lcd();

  #ifdef LOGGING  
    if (telemetry_ok) {
      if (SD_OK && !logging_now && valid_timestamp) {
        start_logging();
      }
    }
    else {
      if (logging_now && !valid_timestamp) {
        stop_logging();
      }
    }

    if ((link_initiated) && ((!rssi_ok) || (!telemetry_ok)))
      tone(BUZZER_PIN ,  100, 500);

  #endif
  
  }
#endif  

  uavtalk_read();

#ifdef OLEDLCD
  if (millis() - lastpacketreceived > 2000) {
    telemetry_ok = false;     
  }
  if (telemetry_ok && rssi_ok) {
    link_initiated = true;
  }
#endif
}


void playTones(uint8_t alertlevel) {
  static int toneCounter = 0;
  toneCounter += 1;
  switch  (toneCounter) {
  case 1:
    tone(BUZZER_PIN ,  1047, 100); 
    break;
  case 4:
    if (alertlevel == 1)
      tone(BUZZER_PIN , 1047,100);
    else if (alertlevel == 2)
      tone(BUZZER_PIN , 1047,500);
    break;
  case 50:
    if (alertlevel == 2) {
      toneCounter = 0; 
    }
    break;
  case 100:
    if (alertlevel == 1) {
      toneCounter = 0; 
    }
    break;
  default: 
    break;
  }
}

/* *********************************************** */
/* ******** functions used in main loop() ******** */

#ifdef LOGGING
void start_logging() {

  char buffer[5];
  char dirname[13];
  char filename[26];

#ifdef USE_WDT
  wdt_reset();
#endif
  if (!logging_now) {

    strcpy(dirname, format_dec_4(osd_time_year, buffer));
    strcat(dirname, format_dec_2(osd_time_month, buffer));
    strcat(dirname, format_dec_2(osd_time_day, buffer));
//    sprintf(dirname, "%02d%02d%04d", osd_time_day, osd_time_month, osd_time_year);
    strcpy(filename, dirname);
    strcat_P(filename, (char *)F("/"));
    strcat(filename, format_dec_2(osd_time_hour, buffer));
    strcat(filename, format_dec_2(osd_time_minute, buffer));
    strcat(filename, format_dec_2(osd_time_second, buffer));
    strcat_P(filename, (char *)F(".opl"));
//    sprintf(filename, "%s/%02d%02d%02d.opl", dirname, osd_time_hour, osd_time_minute, osd_time_second);
    if (SD_OK) {
      if (!SD.exists(dirname)) {
        SD_OK = SD.mkdir(dirname);
      }
    }

    if (SD_OK) {
      if (SD.exists(filename)) {
        SD.remove(filename);
      }

      dataFile = SD.open(filename,  O_CREAT | O_WRITE);
      if (dataFile) {
        logsize = 0;
        logging_now = true;
        logging_start_millis = millis();
      }
      else {
        SD_OK = false;
      }
    }
    
    if (SD_OK) {
      for (int i = 3; i <= 9; i++) {
        tone(BUZZER_PIN, i*100);
      #ifdef USE_WDT
        wdt_reset();
      #endif  
        delay(100);
        noTone(BUZZER_PIN);
      }
    }
    else {
      tone(BUZZER_PIN, 200); 
    #ifdef USE_WDT
      wdt_reset();
    #endif  
      delay(1000);
    #ifdef USE_WDT
      wdt_reset();
    #endif  
      noTone(BUZZER_PIN);
    }  
  }
}

void stop_logging() {

#ifdef USE_WDT
  wdt_reset();
#endif
  if (logging_now) {
    logging_now = false;
    dataFile.close();

    for (int i = 9; i >= 3; i--) {
      tone(BUZZER_PIN, i*100);
    #ifdef USE_WDT
      wdt_reset();
    #endif  
      delay(50);
      noTone(BUZZER_PIN);
    }

    delay(100);
    for (int i = 9; i >= 3; i--) {
      tone(BUZZER_PIN, i*100);
    #ifdef USE_WDT
      wdt_reset();
    #endif  
      delay(50);
      noTone(BUZZER_PIN);
    }
  }
}
#endif


#ifdef STATION_BATTERY_MONITORING
void updateVoltage(void) 
{
  
  float sampled_voltage = CURRENT_VOLTAGE(analogRead(A3));  // reads battery voltage

  if (station_voltage < 0.0) 
    station_voltage = sampled_voltage;
  else 
    station_voltage = sampled_voltage * 0.02 + station_voltage * 0.98; 

  if (station_voltage <= MIN_VOLTAGE2)
  {
    buzzer_status = 2;
  }
  else if (station_voltage <= MIN_VOLTAGE1) {
    buzzer_status = 1;
  }
  else {
    buzzer_status = 0;
  }

#ifdef DEBUG 
  Serial.println(voltage);
#endif    
}
#endif


void setHomeVars() 
{
  float dstlon, dstlat;

  if (osd_got_home) {
    // shrinking factor for longitude going to poles direction
    float rads = fabs(osd_home_lat) * 0.0174532925;
    double scaleLongDown = cos(rads);

    // DST to Home
    dstlat = fabs(osd_home_lat - osd_lat) * 111319.5;
    dstlon = fabs(osd_home_lon - osd_lon) * 111319.5 * scaleLongDown;
    osd_home_distance = sqrt(sq(dstlat) + sq(dstlon));
  } 
  else {
    // criteria for a stable home position:
    // - GPS 3D fix
    // - with at least 5 satellites
    // - osd_alt stable for 30 * 100ms = 3s
    // - osd_alt stable means the delta is lower 0.5m
    if (osd_fix_type > 2 && osd_satellites_visible >= 5 && osd_alt_cnt < 30) {
      if (fabs(osd_alt_prev - osd_alt) > 0.5) {
        osd_alt_cnt  = 0;
        osd_alt_prev = osd_alt;
      } 
      else {
        if (++osd_alt_cnt >= 30) {
          osd_home_lat = osd_lat; // take this osd_lat as osd_home_lat
          osd_home_lon = osd_lon; // take this osd_lon as osd_home_lon

          osd_home_alt = osd_alt; // take this stable osd_alt as osd_home_alt (GPS only)
          osd_got_home = 1;
        }
      }
    }
  }
}

#ifdef OLEDLCD

char* format_dec_2(int val, char *s) {
  char buffer[2];
  strcpy(s, itoa(((val/10)%10), buffer, 10));
  strcat(s, itoa((val%10), buffer, 10));
  return s;
}


char* format_dec_4(int val, char *s) {
  char buffer[2];
  strcpy(s, itoa(((val/1000)%10), buffer, 10));
  strcat(s, itoa(((val/100)%10), buffer, 10));
  strcat(s, itoa(((val/10)%10), buffer, 10));
  strcat(s, itoa((val%10), buffer, 10));
  return s;
}


void refresh_lcd() { 

  display.setCursor(0,0);

  for ( int i = 1 ; i<7; i++ ) {

    char currentline[30];
    char buffer[22];

    switch (i) {
    case 1: 
      if (!telemetry_ok) {
        strcpy_P(currentline, (char *)F("NO TELEMETRY"));
      }
      else {
        strcpy_P(currentline, (char *)F("SATS:"));
        itoa(osd_satellites_visible, buffer, 10);        
        strcat(currentline, buffer);
        strcat_P(currentline, (char *)F(" FIX:"));
        itoa(osd_fix_type, buffer, 10);        
        strcat(currentline, buffer);
//      sprintf(currentline,"SATS:%d FIX:%d ", osd_satellites_visible, osd_fix_type);
      }

    #ifdef STATION_BATTERY_MONITORING
      for ( int l = strlen(currentline); l<15 ; l++ ) {   
        strcat_P(currentline,(char *)F(" "));
      }
      strcat(currentline, dtostrf(station_voltage, 5, 2, buffer));
      strcat_P(currentline, (char *)F("v"));
    #else  
      for ( int l = strlen(currentline); l<21 ; l++ ) {  
        strcat_P(currentline,(char *)F(" "));
      }
    #endif  
      break;
    case 2:
        strcpy_P(currentline, (char *)F("Dist:"));
//        itoa((int16_t)(osd_home_distance), buffer, 10);
        if (osd_home_distance < 1000000)                  // 1000km
          dtostrf(osd_home_distance, 1, 0, buffer);
        else  
          strcpy_P(buffer, (char *)F("???"));
        strcat(currentline, buffer);
        strcat_P(currentline, (char *)F("m Alt:"));
        dtostrf((osd_alt - osd_home_alt), 1, 0, buffer);
//        itoa((int16_t)(revo_baro_alt - osd_baro_home_alt), buffer, 10);        
        strcat(currentline, buffer);
        strcat_P(currentline, (char *)F("m"));

        for ( int l = strlen(currentline); l<21 ; l++ ) {  
          strcat_P(currentline,(char *)F(" "));
        }
//      sprintf(currentline, "Dist:%dm Alt:%dm ", (int16_t)(osd_home_distance), (int16_t)(revo_baro_alt - osd_baro_home_alt));
      break;
    case 3:   
      if (valid_timestamp) {
        strcpy(currentline, format_dec_2(osd_time_day, buffer));
        strcat_P(currentline, (char *)F("."));
        strcat(currentline, format_dec_2(osd_time_month, buffer));
        strcat_P(currentline, (char *)F("."));
        strcat(currentline, format_dec_4(osd_time_year, buffer));
        strcat_P(currentline, (char *)F(" "));
        strcat(currentline, format_dec_2(osd_time_hour, buffer));
        strcat_P(currentline, (char *)F(":"));
        strcat(currentline, format_dec_2(osd_time_minute, buffer));
        strcat_P(currentline, (char *)F(":"));
        strcat(currentline, format_dec_2(osd_time_second, buffer));
      }
      else
        strcpy_P(currentline, (char *)F("--.--.---- --:--:--"));
       
      switch (indicator) {
        case 1:           
          strcpy_P(buffer, (char *)F(" :"));
          indicator = 2;
          break;
        case 2:
          strcpy_P(buffer, (char *)F(" o"));
          indicator = 3;
          break;
        case 3:
          strcpy_P(buffer, (char *)F(" *"));
          indicator = 1;
          break;
      }
      strcat(currentline, buffer);
//      sprintf(currentline, "%02d.%02d.%04d %02d:%02d:%02d ", osd_time_day, osd_time_month, osd_time_year, osd_time_hour, osd_time_minute, osd_time_second);
      break;
    case 4: 
    #ifdef LOGGING
      if (SD_OK) {
        if (logging_now) {
          strcpy_P(currentline, (char *)F("Logging: "));
          strcat(currentline, dtostrf((float)logsize / 1024.0, 1, 1, buffer));
          strcat_P(currentline, (char *)F(" KB"));
//          sprintf(currentline,"Logging: %s KB", dtostrf((float)logsize / 1024.0, 1, 1, bufferz));
        }
        else {
          strcpy_P(currentline, (char *)F("Logged: "));
          strcat(currentline, dtostrf((float)logsize / 1024.0, 1, 1, buffer));
          strcat_P(currentline, (char *)F(" KB "));
//          sprintf(currentline,"Logged: %s KB ", dtostrf((float)logsize / 1024.0, 1, 1, bufferz));
        }
      }
      else
        strcpy_P(currentline, (char *)F("NO LOGGING! SD Fail!"));
//        sprintf(currentline, "%s", "NO LOGGING! SD Fail!"); 
    #else
      strcpy_P(currentline, (char *)F(""));
//      sprintf(currentline, "%s", "");
    #endif   
      break;
    case 5:
      strcpy(currentline, dtostrf(osd_lat, 6, 6, buffer));
      
      for ( int l = strlen(currentline); l<10 ; l++ ) {  
        strcat_P(currentline,(char *)F(" "));
      }
//      sprintf(currentline,"%s", dtostrf(osd_lat, 6, 6, bufferl));
      display.set2X();
      break;
    case 6:
      strcpy(currentline, dtostrf(osd_lon, 6, 6, buffer));

      for ( int l = strlen(currentline); l<10 ; l++ ) {  
        strcat_P(currentline,(char *)F(" "));
      }
//      sprintf(currentline,"%s", dtostrf(osd_lon, 6, 6, bufferL));
      display.set2X();
      break;
    }
    display.println(currentline);
    display.set1X();
  }  
}

#endif


//######################################## WATCHDOG #############################################

#ifdef USE_WDT
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

void wdt_init(void)
{
	MCUSR = 0;
	wdt_disable();
	return;
}

ISR(WDT_vect) // Watchdog timer interrupt.
{
// Include your code here - be careful not to use functions they may cause the interrupt to hang and
// prevent a reset.
}

void watchdogSetup(void)
{
  cli();        // disable all interrupts
  wdt_reset();  // reset the WDT timer
  wdt_disable();

//WDP  WDP  WDP  WDP  Time-out
//3    2    1    0    (ms)

//0    0    0    0    16
//0    0    0    1    32
//0    0    1    0    64
//0    0    1    1    125
//0    1    0    0    250
//0    1    0    1    500
//0    1    1    0    1000
//0    1    1    1    2000
//1    0    0    0    4000
//1    0    0    1    8000 

 
// WDTCSR configuration:
// WDIE = 1: Interrupt Enable
// WDE = 1 :Reset Enable
 
// WDP3 = 0 :For 2000ms Time-out
// WDP2 = 1 :For 2000ms Time-out
// WDP1 = 1 :For 2000ms Time-out
// WDP0 = 1 :For 2000ms Time-out

// Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE);
// Set Watchdog settings:
  WDTCSR = (1<<WDIE) | (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
  sei();
}

#endif
