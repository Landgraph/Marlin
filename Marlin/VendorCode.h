/**
 * Anycubic Vendor Specific Code
 *
 * Based on Anycubic Chiron Codebase
 * Copyright (C) 2018 Anycubic (https://github.com/ANYCUBIC-3D/ANYCUBIC_CHIRON_V1.3.0)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Edited 2020 by Landgraph (http://www.landgraph.ru/, https://github.com/landgraph/)
 */
#ifndef VENDORCODE_H
#define VENDORCODE_H

#include "MyHardwareSerial.h"
#include "MarlinSerial.h"
#include "types.h"

/******************************************************************/
/*                          Marlin_main                           */
/******************************************************************/

#define NEW_SERIAL_PROTOCOL(x) (NewSerial.print(x))
#define NEW_SERIAL_PROTOCOL_F(x, y) (NewSerial.print(x, y))
#define NEW_SERIAL_PROTOCOLPGM(x) (NewSerialprintPGM(PSTR(x)))
#define NEW_SERIAL_(x) (NewSerial.print(x), NewSerial.write('\n'))
#define NEW_SERIAL_PROTOCOLLN(x) (NewSerial.print(x), NewSerial.write('\r'), NewSerial.write('\n'))
#define NEW_SERIAL_PROTOCOLLNPGM(x) (NewSerialprintPGM(PSTR(x)), NewSerial.write('\n'))

#define TFT_SERIAL_START() (NewSerial.write('\r'), NewSerial.write('\n'))
#define TFT_SERIAL_CMD_SEND(x) (NewSerialprintPGM(PSTR(x)), NewSerial.write('\r'), NewSerial.write('\n'))
#define TFT_SERIAL_ENTER() (NewSerial.write('\r'), NewSerial.write('\n'))
#define TFT_SERIAL_SPACE() (NewSerial.write(' '))

const char newErr[] PROGMEM = "ERR ";
const char newSucc[] PROGMEM = "OK";
#define NEW_SERIAL_ERROR_START (NewSerialprintPGM(newErr))
#define NEW_SERIAL_ERROR(x) NEW_SERIAL_PROTOCOL(x)
#define NEW_SERIAL_ERRORPGM(x) NEW_SERIAL_PROTOCOLPGM(x)
#define NEW_SERIAL_ERRORLN(x) NEW_SERIAL_PROTOCOLLN(x)
#define NEW_SERIAL_ERRORLNPGM(x) NEW_SERIAL_PROTOCOLLNPGM(x)

//##define NEW_SERIAL_ECHO_START (NewSerialprintPGM(newSucc))
#define NEW_SERIAL_ECHOLN(x) NEW_SERIAL_PROTOCOLLN(x)
#define NEW_SERIAL_SUCC_START (NewSerialprintPGM(newSucc))
#define NEW_SERIAL_ECHOPAIR(name, value) (serial_echopair_P(PSTR(name), (value)))
#define NEW_SERIAL_ECHOPGM(x) NEW_SERIAL_PROTOCOLPGM(x)
#define NEW_SERIAL_ECHO(x) NEW_SERIAL_PROTOCOL(x)

FORCE_INLINE void NewSerialprintPGM(const char *str)
{
  char ch = pgm_read_byte(str);
  while (ch)
  {
    NewSerial.write(ch);
    ch = pgm_read_byte(++str);
  }
}
void NEWFlushSerialRequestResend();
void NEWClearToSend();

extern char TFTpausingFlag;
extern char TFTresumingflag;
extern char PointTestFlag;
extern char errorFlag;
extern char FlagResumFromOutage;
extern unsigned char ResumingFlag;
extern char sdcardstartprintingflag;
extern char seekdataflag;
extern bool UsbOnLineFlag;
extern bool USBConnectFlag;
#if defined(OutageTest)
extern int PowerInt;
extern unsigned char PowerTestFlag;
#endif
#if PIN_EXISTS(SD_DETECT)
extern uint8_t lcd_sd_status;
#endif

extern const unsigned int Max_ModelCooling;
#if HAS_BED_PROBE
extern float NEW_zprobe_zoffset;
#endif
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
extern void setupMyZoffset();
#endif

extern char *itostr3(const int &x);
extern char *itostr2(const uint8_t &x);
extern void PowerKill();
extern void FilamentScan();
extern void Fan2Scan();
extern void TFT_Commond_Scan();
extern void setup_OutageTestPin();
extern void SetUpFAN2_PIN();
extern void setupSDCARD();
extern void SetupFilament();
extern uint16_t MyGetFileNr();
extern void USBOnLineTest();
extern void SDCARD_UPDATA();
extern void pauseCMDsend();

#if ENABLED(SDSUPPORT)
#if defined(OutageTest)
extern unsigned char PowerTestFlag;
extern char seekdataflag;
#endif
extern char TFTStatusFlag;
extern char sdcardstartprintingflag;
extern uint16_t filenumber;
extern bool pauseCMDsendflag;
#endif

/******************************************************************/
/*                      configuration_store                       */
/******************************************************************/

#if ENABLED(EEPROM_SETTINGS)
void SaveAutoBedGridData();
void ReadAutoBedGridData();

#ifdef OutageTest
extern bool RestartFlag;
void OutageSave();
void OutageRead();
extern float last_position[4];
extern long last_sd_position[1];
#endif
extern float Current_z_offset;
extern float last_z_offset;
extern unsigned char FirstBootFlag;
void SaveFirstBootFlag();
void readFirstBootFlag();

extern float Current_z_offset;

void SaveWay2Leveling();
void ReadWay2Leveling();
extern unsigned char Manual_Leveling;
extern unsigned char FirstBootFlag;
void SaveFirstBootFlag();
void readFirstBootFlag();
#endif //ENABLED(EEPROM_SETTINGS)

#endif //VENDORCODE_H
