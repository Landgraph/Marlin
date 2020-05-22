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

#include "Marlin.h"

#ifdef VENDOR_CODE

#include "stepper.h"
#include "temperature.h"
#include "configuration_store.h"
#include "music.h"
#include "ultralcd.h"

#if ENABLED(SDSUPPORT)
#include "cardreader.h"

extern CardReader card; //defined in Marlin_main.cpp
#endif

#if HAS_BED_PROBE
float NEW_zprobe_zoffset;
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
extern int bilinear_grid_spacing[2], bilinear_start[2];
extern float z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
#endif

extern long gcode_N, gcode_LastN, Stopped_gcode_LastN;
extern uint8_t cmd_queue_index_r,
    cmd_queue_index_w,
    commands_in_queue;

char seekdataflag = 0;
//char AssistLeveTestflag=0;
char PointTestFlag = 0;
char Z_offset_debug_flag = 0;
float Current_z_offset;
//static bool HomeFlag=0;
//char AutoLevelLowSpeedModelFlag=0;
uint16_t filenumber;
bool USBConnectFlag = 0;
bool ReadMyfileNrFlag = true;
extern uint16_t MyFileNrCnt;

unsigned char Manual_Leveling;

static char TFTcmdbuffer[TFTBUFSIZE][TFT_MAX_CMD_SIZE];
static int TFTbuflen = 0;
static int TFTbufindr = 0;
static int TFTbufindw = 0;
static bool TFTfromsd[TFTBUFSIZE];
static char serial3_char;
static int serial3_count = 0;
static boolean TFTcomment_mode = false;
static char *TFTstrchr_pointer;

unsigned long starttime = 0;
unsigned long stoptime = 0;

#if defined(OutageTest)
int PowerInt = 6; //
unsigned char PowerTestFlag = false;
unsigned char ResumingFlag = 0;
bool RestartFlag = false;
#endif

char FlagResumFromOutage = 0;

const unsigned int Max_ModelCooling = MAX_MODEL_COOLING_PRECENT_VALUE * 255;

char TFTpausingFlag = 0; //for return a flag that buffer carry out
char TFTStatusFlag = 0;
char TFTresumingflag = 0;
char sdcardstartprintingflag = 0;
char FilamentTestFlag = true;
//float MYfeedrate_mm_s=2000;

#if PIN_EXISTS(SD_DETECT)
uint8_t lcd_sd_status;
#endif

char errorFlag = 0;

char conv[9];

#define DIGIT(n) ('0' + (n))
#define DIGIMOD(n, f) DIGIT((n) / (f) % 10)
#define RJDIGIT(n, f) ((n) >= (f) ? DIGIMOD(n, f) : ' ')
#define MINUSOR(n, alt) (n >= 0 ? (alt) : (n = -n, '-'))

// Convert unsigned int to string with 12 format
char *itostr2(const uint8_t &x)
{
  int xx = x;
  conv[0] = DIGIMOD(xx, 10);
  conv[1] = DIGIMOD(xx, 1);
  conv[2] = '\0';
  return conv;
}

char *itostr3(const int &x)
{
  int xx = x;
  conv[0] = MINUSOR(xx, RJDIGIT(xx, 100));
  conv[1] = RJDIGIT(xx, 10);
  conv[2] = DIGIMOD(xx, 1);
  conv[3] = '\0';
  return conv;
}

void setup_OutageTestPin()
{
#if defined(OutageTest)
  pinMode(OUTAGETEST_PIN, INPUT);
  //  WRITE(OUTAGETEST_PIN,HIGH);
  pinMode(OUTAGECON_PIN, OUTPUT);
  WRITE(OUTAGECON_PIN, LOW);
#endif
}
#define FilamentTestPin 33
void SetupFilament()
{
  pinMode(FilamentTestPin, INPUT);
  WRITE(FilamentTestPin, HIGH);
  _delay_ms(50);
  /*
    if(READ(FilamentTestPin)==true)
    {
      NEW_SERIAL_PROTOCOLPGM("J15");//j15 FILAMENT LACK
      TFT_SERIAL_ENTER();
      FilamentLack();//music
     }
     */
}

void FilamentScan()
{
  static char last_status = READ(FilamentTestPin);
  static unsigned char now_status, status_flag = false;
  static unsigned int counter = 0;
  now_status = READ(FilamentTestPin) & 0xff;
  // if (now_status==last_status) return;
  if (now_status > last_status)
  {
    counter++;
    if (counter >= 50000)
    {
      counter = 0;
      FilamentLack(); //music
      if ((card.sdprinting == true))
      {
        NEW_SERIAL_PROTOCOLPGM("J23"); //j23 FILAMENT LACK with the prompt box don't disappear
        TFT_SERIAL_ENTER();
        TFTpausingFlag = true;
        card.pauseSDPrint();
      }
      else if ((card.sdprinting == false))
      {
        NEW_SERIAL_PROTOCOLPGM("J15"); //j15 FILAMENT LACK
        TFT_SERIAL_ENTER();
      }
      last_status = now_status;
    }
  }
  else if (now_status != last_status)
  {
    counter = 0;
    last_status = now_status;
  }
  //  else  {counter=0;last_status=now_status;}
}

bool UsbOnLineFlag = false;
void USBOnLineTest()
{
  static long int temp = 0;
  if (USBConnectFlag == false)
  {
    if (UsbOnLineFlag == true)
    {
      temp++;
      UsbOnLineFlag = false;
      if (temp > 1)
      {
        USBConnectFlag = true;
        NEW_SERIAL_PROTOCOLPGM("J03"); //usb connect
        TFT_SERIAL_ENTER();
        temp = 0;
      }
    }
  }
  else if (USBConnectFlag == true)
  {
    if (UsbOnLineFlag == false)
    {
      temp++;
      if (temp > 50000)
      {
        UsbOnLineFlag = false;
        USBConnectFlag = false;
        NEW_SERIAL_PROTOCOLPGM("J12"); //ready
        TFT_SERIAL_ENTER();
        temp = 0;
      }
    }
    else
    {
      temp = 0;
      UsbOnLineFlag = false;
    }
  }
}

void PowerKill()
{
  //  SERIAL_ECHOLNPGM("int17 be called");
#ifdef OutageTest
  if (PowerTestFlag == true)
  {
    //		 MYfeedrate_mm_s=feedrate_mm_s;
    thermalManager.disable_all_heaters();
    //  #ifdef OutageTest
    //  OutageSave();
    //  #endif
    disable_X();
    disable_Y();
    disable_Z();
    disable_e_steppers();
    OutageSave();
    PowerTestFlag = false;
  }
#endif
}

#define Z_TEST 2
//#define BEEPER_PIN 31
void setuplevelTest()
{
  pinMode(Z_TEST, INPUT);
  WRITE(Z_TEST, HIGH);
  pinMode(BEEPER_PIN, OUTPUT);
  WRITE(BEEPER_PIN, LOW);
}

void Newok_to_send()
{
  previous_move_ms = millis();
  /*
  if (!send_ok[cmd_queue_index_r]) return;
 // SERIAL_PROTOCOLPGM(MSG_OK);
  #if ENABLED(ADVANCED_OK)
    char* p = command_queue[cmd_queue_index_r];
    if (*p == 'N') {
      SERIAL_PROTOCOL(' ');
      SERIAL_ECHO(*p++);
      while (NUMERIC_SIGNED(*p))
        SERIAL_ECHO(*p++);
    }
    SERIAL_PROTOCOLPGM(" P"); SERIAL_PROTOCOL(int(BLOCK_BUFFER_SIZE - planner.movesplanned() - 1));
    SERIAL_PROTOCOLPGM(" B"); SERIAL_PROTOCOL(BUFSIZE - commands_in_queue);
  #endif
  SERIAL_EOL;
  */
}
void NEWFlushSerialRequestResend()
{
  //char command_queue[cmd_queue_index_r][100]="Resend:";
  NewSerial.flush();
  // SERIAL_PROTOCOLPGM(MSG_RESEND);
  //  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  Newok_to_send();
}

float TFTcode_value()
{
  return (strtod(&TFTcmdbuffer[TFTbufindr][TFTstrchr_pointer - TFTcmdbuffer[TFTbufindr] + 1], NULL));
}
bool TFTcode_seen(char code)
{
  TFTstrchr_pointer = strchr(TFTcmdbuffer[TFTbufindr], code);
  return (TFTstrchr_pointer != NULL); //Return True if a character was found
}

uint16_t MyGetFileNr()
{
  if (card.cardOK)
  {
    MyFileNrCnt = 0;
    ReadMyfileNrFlag = true;
    delay(10);
    card.Myls();
  }
  return MyFileNrCnt;
}

void z_offset_auto_test()
{
  float i = 0;
  while ((READ(Z_TEST) == 0))

  {
    i = i + 0.025;
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], i, 0, 6, 2);
    planner.synchronize();
    delay(30);
  }
  Current_z_offset = i;
  //  SaveMyZoffset();
  NEW_SERIAL_PROTOCOLPGM("A9V ");
  NEW_SERIAL_PROTOCOL(int(Current_z_offset * 100));
  TFT_SERIAL_ENTER();
  return;
}

void get_command_from_TFT()
{
  char *starpos = NULL;
  while (NewSerial.available() > 0 && TFTbuflen < TFTBUFSIZE)
  {
    serial3_char = NewSerial.read();
    if (serial3_char == '\n' ||
        serial3_char == '\r' ||
        (serial3_char == ':' && TFTcomment_mode == false) ||
        serial3_count >= (TFT_MAX_CMD_SIZE - 1))
    {
      if (!serial3_count)
      {                          //if empty line
        TFTcomment_mode = false; //for new command
        return;
      }
      TFTcmdbuffer[TFTbufindw][serial3_count] = 0; //terminate string
                                                   // NEW_SERIAL_PROTOCOL("OK:");
                                                   // NEW_SERIAL_PROTOCOLLN(TFTcmdbuffer[TFTbufindw]);
      if (!TFTcomment_mode)
      {
        TFTcomment_mode = false; //for new command
        TFTfromsd[TFTbufindw] = false;
        if (strchr(TFTcmdbuffer[TFTbufindw], 'N') != NULL)
        {
          TFTstrchr_pointer = strchr(TFTcmdbuffer[TFTbufindw], 'N');
          gcode_N = (strtol(&TFTcmdbuffer[TFTbufindw][TFTstrchr_pointer - TFTcmdbuffer[TFTbufindw] + 1], NULL, 10));
          if (gcode_N != gcode_LastN + 1 && (strstr_P(TFTcmdbuffer[TFTbufindw], PSTR("M110")) == NULL))
          {
            NEW_SERIAL_ERROR_START;
            //     NEW_SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
            //     NEW_SERIAL_ERRORLN(gcode_LastN);
            NEWFlushSerialRequestResend();
            serial3_count = 0;
            return;
          }

          if (strchr(TFTcmdbuffer[TFTbufindw], '*') != NULL)
          {
            byte checksum = 0;
            byte count = 0;
            while (TFTcmdbuffer[TFTbufindw][count] != '*')
              checksum = checksum ^ TFTcmdbuffer[TFTbufindw][count++];
            TFTstrchr_pointer = strchr(TFTcmdbuffer[TFTbufindw], '*');

            if ((int)(strtod(&TFTcmdbuffer[TFTbufindw][TFTstrchr_pointer - TFTcmdbuffer[TFTbufindw] + 1], NULL)) != checksum)
            {
              NEW_SERIAL_ERROR_START;
              //     NEW_SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
              //     NEW_SERIAL_ERRORLN(gcode_LastN);
              NEWFlushSerialRequestResend();

              NEW_SERIAL_ERROR_START;
              //     NEW_SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
              //     NEW_SERIAL_ERRORLN(gcode_LastN);
              NEWFlushSerialRequestResend();
              serial3_count = 0;
              return;
            }
            //if no errors, continue parsing
          }
          else
          {
            NEW_SERIAL_ERROR_START;
            //   NEW_SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
            //   NEW_SERIAL_ERRORLN(gcode_LastN);
            NEWFlushSerialRequestResend();
            serial3_count = 0;
            return;
          }
          gcode_LastN = gcode_N;
          //if no errors, continue parsing
        }
        else // if we don't receive 'N' but still see '*'
        {
          if ((strchr(TFTcmdbuffer[TFTbufindw], '*') != NULL))
          {
            NEW_SERIAL_ERROR_START;
            //     NEW_SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
            //     NEW_SERIAL_ERRORLN(gcode_LastN);
            serial3_count = 0;
            return;
          }
        }
        if ((strchr(TFTcmdbuffer[TFTbufindw], 'A') != NULL))
        {
          TFTstrchr_pointer = strchr(TFTcmdbuffer[TFTbufindw], 'A');
          switch ((int)((strtod(&TFTcmdbuffer[TFTbufindw][TFTstrchr_pointer - TFTcmdbuffer[TFTbufindw] + 1], NULL))))
          {

          case 0: //A0 GET HOTEND TEMP
            NEW_SERIAL_PROTOCOLPGM("A0V ");
            NEW_SERIAL_PROTOCOL(itostr3(int(thermalManager.degHotend(0) + 0.5)));
            TFT_SERIAL_ENTER();
            break;
          case 1: //A1  GET HOTEND TARGET TEMP
            NEW_SERIAL_PROTOCOLPGM("A1V ");
            NEW_SERIAL_PROTOCOL(itostr3(int(thermalManager.degTargetHotend(0) + 0.5)));
            TFT_SERIAL_ENTER();
            break;
          case 2: //A2 GET HOTBED TEMP
            NEW_SERIAL_PROTOCOLPGM("A2V ");
            NEW_SERIAL_PROTOCOL(itostr3(int(thermalManager.degBed() + 0.5)));
            TFT_SERIAL_ENTER();
            break;
          case 3: //A3 GET HOTBED TARGET TEMP
            NEW_SERIAL_PROTOCOLPGM("A3V ");
            NEW_SERIAL_PROTOCOL(itostr3(int(thermalManager.degTargetBed() + 0.5)));
            TFT_SERIAL_ENTER();
            break;

          case 4: //A4 GET FAN SPEED
          {
            unsigned int temp;
            temp = ((fanSpeeds[0] * 100) / Max_ModelCooling + 1);
            //   temp=((fanSpeeds[0]*100)/179+1);
            temp = constrain(temp, 0, 100);
            NEW_SERIAL_PROTOCOLPGM("A4V ");
            NEW_SERIAL_PROTOCOL(temp);
            TFT_SERIAL_ENTER();
          }
          break;
          case 5: // A5 GET CURRENT COORDINATE
            NEW_SERIAL_PROTOCOLPGM("A5V");
            TFT_SERIAL_SPACE();
            NEW_SERIAL_PROTOCOLPGM("X: ");
            NEW_SERIAL_PROTOCOL(current_position[X_AXIS]);
            TFT_SERIAL_SPACE();
            NEW_SERIAL_PROTOCOLPGM("Y: ");
            NEW_SERIAL_PROTOCOL(current_position[Y_AXIS]);
            TFT_SERIAL_SPACE();
            NEW_SERIAL_PROTOCOLPGM("Z: ");
            NEW_SERIAL_PROTOCOL(current_position[Z_AXIS]);
            TFT_SERIAL_SPACE();
            TFT_SERIAL_ENTER();
            break;
          case 6: //A6 GET SD CARD PRINTING STATUS
            if (card.sdprinting)
            {
              NEW_SERIAL_PROTOCOLPGM("A6V ");
              TFTStatusFlag = 1;
              //  card.getStatus();
              card.TFTgetStatus();
            }
            else
              NEW_SERIAL_PROTOCOLPGM("A6V ---");
            TFT_SERIAL_ENTER();
            break;
          case 7: //A7 GET PRINTING TIME
          {
            NEW_SERIAL_PROTOCOLPGM("A7V ");
            if (starttime != 0) //print time
            {
              //   uint16_t time = millis()/60000 - starttime/60000;
              uint16_t time = millis() / 60000 - starttime / 60000;
              NEW_SERIAL_PROTOCOL(itostr2(time / 60));
              TFT_SERIAL_SPACE();
              NEW_SERIAL_PROTOCOLPGM("H");
              TFT_SERIAL_SPACE();
              NEW_SERIAL_PROTOCOL(itostr2(time % 60));
              TFT_SERIAL_SPACE();
              NEW_SERIAL_PROTOCOLPGM("M");
            }
            else
            {
              TFT_SERIAL_SPACE();
              NEW_SERIAL_PROTOCOLPGM("999:999");
            }
            TFT_SERIAL_ENTER();
            /*
                          else if(USBConnectFlag)  {
                          stoptime=millis();
                          char time[30];
                          unsigned long t=(stoptime-starttime)/1000;
                          int sec,min;
                          min=t/60;
                          sec=t%60;
                          sprintf_P(time, PSTR("%i M, %i S"), min, sec);
                          TFT_SERIAL_START();
                          NEW_SERIAL_ECHOLN(time);
                          TFT_SERIAL_ENTER();
                          }
                          //autotempShutdown();
                         */

            break;
          }
          case 8: //A8 GET  SD LIST
            MyFileNrCnt = 0;
            if (!IS_SD_INSERTED())
            {
              NEW_SERIAL_PROTOCOLPGM("J02");
              TFT_SERIAL_ENTER();
            }
            else
            {
              MyGetFileNr();
              ReadMyfileNrFlag = false;
              if (TFTcode_seen('S'))
                filenumber = TFTcode_value();
              NEW_SERIAL_PROTOCOLPGM("FN ");
              TFT_SERIAL_ENTER();
              card.Myls();
              NEW_SERIAL_PROTOCOLPGM("END");
              TFT_SERIAL_ENTER();
            }
            break;
          case 9: // a9 pasue sd
            if (card.sdprinting)
            {
              //              static unsigned long lastcounter=0,newcounter=0;  //in case two times Pause of  Commond are too short to lose commond
              //             lastcounter=newcounter;
              //              newcounter=millis();
              //              if(((newcounter-lastcounter)<20000)) break;   //about 15s
              //              else{
              TFTpausingFlag = true;
              card.pauseSDPrint();
              NEW_SERIAL_PROTOCOLPGM("J05"); //j05 pausing
              TFT_SERIAL_ENTER();
              //               }
            }
            else
            {
              NEW_SERIAL_PROTOCOLPGM("J16"); //j16,if status error, send stop print flag in case TFT no response
              TFT_SERIAL_ENTER();
            }
            break;
          case 10: // A10 resume sd print
            if (TFTresumingflag)
            {
              card.startFileprint();
              NEW_SERIAL_PROTOCOLPGM("J04"); //j4ok printing form sd card
              TFT_SERIAL_ENTER();
            }
            break;
          case 11: //A11 STOP SD PRINT
            if ((card.sdprinting) || TFTresumingflag)
            {
              FlagResumFromOutage = 0; //must clean the flag.
              card.TFTStopPringing();
              enqueue_and_echo_commands_P(PSTR("M84"));
            }
            break;
          case 12: //a12 kill
                   //   NEW_SERIAL_PROTOCOLPGM("J11");//kill()
                   //    TFT_SERIAL_ENTER();
                   //    kill();
            break;
          case 13: //A13 SELECTION FILE
                   //if((!USBConnectFlag)&&(!card.sdprinting))
            if ((!planner.movesplanned()) && (!TFTresumingflag))
            {
              starpos = (strchr(TFTstrchr_pointer + 4, '*'));
              if (starpos != NULL)
                *(starpos - 1) = '\0';
              card.openFile(TFTstrchr_pointer + 4, true);
              sdcardstartprintingflag = 1;
              TFT_SERIAL_ENTER();
            }
            break;
          case 14: //A14 START PRINTING
                   // if((!USBConnectFlag)&&(!card.sdprinting))
            if ((!planner.movesplanned()) && (!TFTresumingflag))
            {
              errorFlag = 0;
              card.startFileprint();
              starttime = millis();
              PointTestFlag = false;
              NEW_SERIAL_PROTOCOLPGM("J06"); //hotend heating
              TFT_SERIAL_ENTER();
            }
            break;
          case 15: //A15 RESUMING FROM OUTAGE
                   // if((!USBConnectFlag)&&(!card.sdprinting))
            if ((!planner.movesplanned()) && (!TFTresumingflag))
            {
              if (card.cardOK)
                FlagResumFromOutage = true;
              ResumingFlag = 1;
              card.startFileprint();
              starttime = millis();
              NEW_SERIAL_SUCC_START;
            }
            TFT_SERIAL_ENTER();
            break;
          case 16: //a16 set hotend temp
          {
            unsigned int tempvalue;
            //   char value[15];
            if (TFTcode_seen('S'))
            {
              tempvalue = constrain(TFTcode_value(), 0, 275);
              thermalManager.setTargetHotend(tempvalue, 0);
              //  thermalManager.start_watching_heater(0);
            }
            else if ((TFTcode_seen('C')) && (!planner.movesplanned()))
            {
              //   if((READ(Z_TEST)==0)) enqueue_and_echo_commands_P(PSTR("G1 Z10")); //RASE Z AXIS
              tempvalue = constrain(TFTcode_value(), 0, 275);
              thermalManager.setTargetHotend(tempvalue, 0);
              //   thermalManager.start_watching_heater(0);
            }
          }
          //    TFT_SERIAL_ENTER();
          break;
          case 17: // a17 set hotbed temp
          {
            unsigned int tempbed;
            if (TFTcode_seen('S'))
            {
              tempbed = constrain(TFTcode_value(), 0, 150);
              thermalManager.setTargetBed(tempbed);
              // thermalManager.start_watching_heater(0);
            }
          }
          //  TFT_SERIAL_ENTER();
          break;
          case 18: //a18 set fan speed
            unsigned int temp;
            if (TFTcode_seen('S'))
            {
              unsigned int test = 179;
              temp = (TFTcode_value() * Max_ModelCooling / 100);
              temp = constrain(temp, 0, Max_ModelCooling);
              //     temp=(TFTcode_value()*179/100);
              //    temp=constrain(temp,0,179);
              fanSpeeds[0] = temp;
            }
            else
              fanSpeeds[0] = Max_ModelCooling; //fanSpeeds[0]=179;
            TFT_SERIAL_ENTER();
            break;
          case 19: // A19 CLOSED STEPER DIRV
            if ((!USBConnectFlag) && (!card.sdprinting))
            //  if((!planner.movesplanned())&&(!TFTresumingflag))
            {
              quickstop_stepper();
              disable_X();
              disable_Y();
              disable_Z();
              disable_e_steppers();
            }
            TFT_SERIAL_ENTER();
            break;
          case 20: // a20 read printing speed
          {

            if (TFTcode_seen('S'))
            {
              feedrate_percentage = constrain(TFTcode_value(), 40, 999);
            }
            else
            {
              NEW_SERIAL_PROTOCOLPGM("A20V ");
              NEW_SERIAL_PROTOCOL(feedrate_percentage);
              TFT_SERIAL_ENTER();
            }
          }
          break;
          case 21: //a21 all home
                   // if((!USBConnectFlag)&&(!card.sdprinting))
            if ((!planner.movesplanned()) && (!TFTresumingflag))
            {
              if (TFTcode_seen('X') || TFTcode_seen('Y') || TFTcode_seen('Z'))
              {
                if (TFTcode_seen('X'))
                  enqueue_and_echo_commands_P(PSTR("G28 X"));
                if (TFTcode_seen('Y'))
                  enqueue_and_echo_commands_P(PSTR("G28 Y"));
                if (TFTcode_seen('Z'))
                  enqueue_and_echo_commands_P(PSTR("G28 Z"));
              }
              else if (TFTcode_seen('C'))
                enqueue_and_echo_commands_P(PSTR("G28"));
            }
            break;
          case 22: // A22 move X /Y/Z
                   // if((!USBConnectFlag)&&(!card.sdprinting))
            if ((!planner.movesplanned()) && (!TFTresumingflag))
            {
              float coorvalue;
              unsigned int movespeed = 0;
              char value[30];
              if (TFTcode_seen('F'))
                movespeed = TFTcode_value(); //movespeed=constrain(TFTcode_value(), 1,5000);
              enqueue_and_echo_commands_P(PSTR("G91"));

              if (TFTcode_seen('X'))
              {
                coorvalue = TFTcode_value();
                if ((coorvalue <= 0.2) && coorvalue > 0)
                {
                  sprintf_P(value, PSTR("G1 X0.1F%i"), movespeed);
                  enqueue_and_echo_command_now(value);
                }
                else if ((coorvalue <= -0.1) && coorvalue > -1)
                {
                  sprintf_P(value, PSTR("G1 X-0.1F%i"), movespeed);
                  enqueue_and_echo_command_now(value);
                }
                else
                {
                  sprintf_P(value, PSTR("G1 X%iF%i"), int(coorvalue), movespeed);
                  enqueue_and_echo_command_now(value);
                }
              }
              else if (TFTcode_seen('Y'))
              {
                coorvalue = TFTcode_value();
                if ((coorvalue <= 0.2) && coorvalue > 0)
                {
                  sprintf_P(value, PSTR("G1 Y0.1F%i"), movespeed);
                  enqueue_and_echo_command_now(value);
                }
                else if ((coorvalue <= -0.1) && coorvalue > -1)
                {
                  sprintf_P(value, PSTR("G1 Y-0.1F%i"), movespeed);
                  enqueue_and_echo_command_now(value);
                }
                else
                {
                  sprintf_P(value, PSTR("G1 Y%iF%i"), int(coorvalue), movespeed);
                  enqueue_and_echo_command_now(value);
                }
              }
              else if (TFTcode_seen('Z'))
              {
                coorvalue = TFTcode_value();
                if ((coorvalue <= 0.2) && coorvalue > 0)
                {
                  sprintf_P(value, PSTR("G1 Z0.1F%i"), movespeed);
                  enqueue_and_echo_command_now(value);
                }
                else if ((coorvalue <= -0.1) && coorvalue > -1)
                {
                  sprintf_P(value, PSTR("G1 Z-0.1F%i"), movespeed);
                  enqueue_and_echo_command_now(value);
                }
                else
                {
                  sprintf_P(value, PSTR("G1 Z%iF%i"), int(coorvalue), movespeed);
                  enqueue_and_echo_command_now(value);
                }
              }
              else if (TFTcode_seen('E'))
              {
                coorvalue = TFTcode_value();
                if ((coorvalue <= 0.2) && coorvalue > 0)
                {
                  sprintf_P(value, PSTR("G1 E0.1F%i"), movespeed);
                  enqueue_and_echo_command_now(value);
                }
                else if ((coorvalue <= -0.1) && coorvalue > -1)
                {
                  sprintf_P(value, PSTR("G1 E-0.1F%i"), movespeed);
                  enqueue_and_echo_command_now(value);
                }
                else
                {
                  sprintf_P(value, PSTR("G1 E%iF500"), int(coorvalue));
                  enqueue_and_echo_command_now(value);
                }
                //  else {sprintf_P(value,PSTR("G1 E%iF%i"),int(coorvalue),movespeed); enqueue_and_echo_command_now(value); }
              }
              enqueue_and_echo_commands_P(PSTR("G90"));
            }
            TFT_SERIAL_ENTER();
            break;
          case 23: //a23 prheat pla
                   // if((!USBConnectFlag)&&(!card.sdprinting))
            if ((!planner.movesplanned()) && (!TFTresumingflag))
            {
              //    if((READ(Z_TEST)==0)) enqueue_and_echo_commands_P(PSTR("G1 Z10")); //RASE Z AXIS
              thermalManager.setTargetBed(50);
              thermalManager.setTargetHotend(190, 0);
              //     enqueue_and_echo_commands_P(PSTR("M140 S50")); //HOTBED
              //     enqueue_and_echo_commands_P(PSTR("M104 S190")); //HOTBED
              NEW_SERIAL_SUCC_START;
              TFT_SERIAL_ENTER();
            }
            break;
          case 24: //a24 prheat abs
                   // if((!USBConnectFlag)&&(!card.sdprinting))
            if ((!planner.movesplanned()) && (!TFTresumingflag))
            {
              //      if((READ(Z_TEST)==0)) enqueue_and_echo_commands_P(PSTR("G1 Z10")); //RASE Z AXIS
              thermalManager.setTargetBed(80);
              thermalManager.setTargetHotend(240, 0);
              //     enqueue_and_echo_commands_P(PSTR("M140 S80")); //HOTBED
              //    enqueue_and_echo_commands_P(PSTR("M104 S240")); //HOTBED
              NEW_SERIAL_SUCC_START;
              TFT_SERIAL_ENTER();
            }
            break;
          case 25: //a25 cool down
                   //if((!USBConnectFlag)&&(!card.sdprinting))
            if ((!planner.movesplanned()) && (!TFTresumingflag))
            {
              thermalManager.setTargetHotend(0, 0);
              thermalManager.setTargetBed(0);
              NEW_SERIAL_PROTOCOLPGM("J12"); //
              TFT_SERIAL_ENTER();
            }
            break;
          case 26: //a26 refresh
            card.initsd();
            if (!IS_SD_INSERTED())
            {
              NEW_SERIAL_PROTOCOLPGM("J02");
              TFT_SERIAL_ENTER();
            }
            //       else enqueue_and_echo_commands_P(PSTR("M20"));
            //        TFT_SERIAL_ENTER();
            break;
#ifdef SERVO_ENDSTOPS
          case 27: //a27 servos angles  adjust
                   //if((!USBConnectFlag)&&(!card.sdprinting))
            if ((!planner.movesplanned()) && (!TFTresumingflag))
            {
              char value[30];
              planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], 20, current_position[E_AXIS], 10, active_extruder);
              planner.synchronize();
              NEW_SERIAL_PROTOCOLPGM("A27V ");
              NEW_SERIAL_PROTOCOLPGM("R ");
              NEW_SERIAL_PROTOCOL(RiseAngles);
              TFT_SERIAL_SPACE();
              NEW_SERIAL_PROTOCOLPGM("F ");
              NEW_SERIAL_PROTOCOL(FallAngles);
              TFT_SERIAL_SPACE();
              if (TFTcode_seen('R'))
              {
                RiseAngles = TFTcode_value();
              }
              if (TFTcode_seen('F'))
              {
                FallAngles = TFTcode_value();
              }
              if (TFTcode_seen('O'))
              {
                SaveMyServoAngles();
                delay(200);
                servos[0].detach();
              }
            }
            TFT_SERIAL_ENTER();
            break;
#endif
          case 28: //A28 filament test
          {
            if (TFTcode_seen('O'))
              ;
            else if (TFTcode_seen('C'))
              ;
          }
            TFT_SERIAL_ENTER();
            break;
#ifdef AUTO_BED_LEVELING_BILINEAR
          case 29: //A29 bed grid read
          {
            unsigned char temp_x = 0, temp_y = 0;
            if (TFTcode_seen('X'))
              temp_x = TFTcode_value();
            if (TFTcode_seen('Y'))
              temp_y = TFTcode_value();
            float Zvalue = z_values[temp_x][temp_y];
            Zvalue = Zvalue * 100;
            NEW_SERIAL_PROTOCOLPGM("A29V ");
            NEW_SERIAL_PROTOCOL(Zvalue);
            TFT_SERIAL_ENTER();
          }
          break;

          case 30: //a30 auto leveling
          {
            if (Manual_Leveling == 0xaa)
            {
              NEW_SERIAL_PROTOCOLPGM("J24"); // forbid auto leveling
              TFT_SERIAL_ENTER();
              break;
            }
            if ((planner.movesplanned()) || (card.sdprinting))
            {
              NEW_SERIAL_PROTOCOLPGM("J24"); // forbid auto leveling
              TFT_SERIAL_ENTER();
            }
            else
            {
              NEW_SERIAL_PROTOCOLPGM("J26"); //start auto leveling
              TFT_SERIAL_ENTER();
            }
            if (TFTcode_seen('S'))
            {
              Manual_Leveling = 0x55;
              enqueue_and_echo_commands_P(PSTR("G28\nG29"));
            }
          }
          break;
          case 31: //a31 zoffset set get or save
          {
            if (Manual_Leveling == 0xaa)
              break;
            if (TFTcode_seen('S'))
            {
              float value = constrain(TFTcode_value(), -1.0, 1.0);
              NEW_zprobe_zoffset += value;
              for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
              {
                for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
                  z_values[x][y] += value;
              }
              NEW_SERIAL_PROTOCOLPGM("A31V ");
              NEW_SERIAL_PROTOCOL(NEW_zprobe_zoffset);
              TFT_SERIAL_ENTER();
            }
            if (TFTcode_seen('G'))
            {
              NEW_SERIAL_PROTOCOLPGM("A31V ");
              NEW_SERIAL_PROTOCOL(NEW_zprobe_zoffset);
              TFT_SERIAL_ENTER();
            }
            if (TFTcode_seen('D'))
              SaveAutoBedGridData();
          }
            TFT_SERIAL_ENTER();
            break;
#endif
          case 32: //a32 clean leveling beep flag
                   //   {
                   //       PointTestFlag=0;
                   //    }
            break;
          case 33: // a33 get version info
          {
            if (errorFlag == 0)
            {
              NEW_SERIAL_PROTOCOLPGM("J33 ");
              NEW_SERIAL_PROTOCOLPGM(MSG_MY_VERSION);
              TFT_SERIAL_ENTER();
            }
            else if (errorFlag == 1)
            {
              NEW_SERIAL_PROTOCOLPGM("J33 ");
              NEW_SERIAL_PROTOCOLPGM("ReadSD card error!");
              TFT_SERIAL_ENTER();
            }
            else if (errorFlag == 2)
            {
              NEW_SERIAL_PROTOCOLPGM("J33 ");
              NEW_SERIAL_PROTOCOLPGM("MinT0");
              TFT_SERIAL_ENTER();
            }
            else if (errorFlag == 3)
            {
              NEW_SERIAL_PROTOCOLPGM("J33 ");
              NEW_SERIAL_PROTOCOLPGM("MinT1");
              TFT_SERIAL_ENTER();
            }
            else if (errorFlag == 4)
            {
              NEW_SERIAL_PROTOCOLPGM("J33 ");
              NEW_SERIAL_PROTOCOLPGM("MaxT0");
              TFT_SERIAL_ENTER();
            }
            else if (errorFlag == 5)
            {
              NEW_SERIAL_PROTOCOLPGM("J33 ");
              NEW_SERIAL_PROTOCOLPGM("MaxT1");
              TFT_SERIAL_ENTER();
            }
            else if (errorFlag == 6)
            {
              NEW_SERIAL_PROTOCOLPGM("J33 ");
              NEW_SERIAL_PROTOCOLPGM("Killed");
              TFT_SERIAL_ENTER();
            }
          }
          break;
#ifdef AUTO_BED_LEVELING_BILINEAR
          case 34: //a34 bed grid write
          {
            uint8_t x_array = 0, y_array = 0, result = 0;
            if (Manual_Leveling == 0xaa)
              break;
            if (TFTcode_seen('X'))
              x_array = constrain(TFTcode_value(), 0, GRID_MAX_POINTS_X);
            if (TFTcode_seen('Y'))
              y_array = constrain(TFTcode_value(), 0, GRID_MAX_POINTS_Y);
            if (TFTcode_seen('V'))
            {
              float i = constrain(TFTcode_value() / 100, -10, 10);
              z_values[x_array][y_array] = i;
            }
            if (TFTcode_seen('S'))
              SaveAutoBedGridData();
            if (TFTcode_seen('C'))
              ReadAutoBedGridData(); // if click return(didn't choose save),needs restore bed grid data.
          }
          break;
#endif
          default:
            break;
          }
        }
        TFTbufindw = (TFTbufindw + 1) % TFTBUFSIZE;
        TFTbuflen += 1;
      }
      serial3_count = 0; //clear buffer
    }
    else
    {
      if (serial3_char == ';')
        TFTcomment_mode = true;
      if (!TFTcomment_mode)
        TFTcmdbuffer[TFTbufindw][serial3_count++] = serial3_char;
    }
  }
}

void mybeep(int beepP, int beepS)
{
  if (beepS > 0)
  {
#if BEEPER_PIN > 0
    tone(BEEPER_PIN, beepS);
    delay(beepP);
    noTone(BEEPER_PIN);
#elif defined(ULTRALCD)
    lcd_buzz(beepS, beepP);
#elif defined(LCD_USE_I2C_BUZZER)
    lcd_buzz(beepP, beepS);
#endif
  }
  else
  {
    delay(beepP);
  }
}

void Endstopsbeep()
{
  static char last_status = ((READ(X_MIN_PIN) << 3) | (READ(Y_MIN_PIN) << 2) | (READ(Z_MAX_PIN) << 1) | READ(Z_MIN_PIN));
  static unsigned char now_status, status_flag = false, counter = 0;

  now_status = ((READ(X_MIN_PIN) << 3) | (READ(Y_MIN_PIN) << 2) | (READ(Z_MAX_PIN) << 1) | READ(Z_MIN_PIN)) & 0xff;
  if (now_status < last_status)
  {
    counter++;
    if (counter >= 250)
    {
      counter = 0;
      mybeep(60, 2000);
      last_status = now_status;
    }
  }
  else if (now_status != last_status)
  {
    counter = 0;
    last_status = now_status;
  }
}
#ifdef AUTO_BED_LEVELING_BILINEAR
void setupMyZoffset()
{
  //  ReadMyZoffset();
  readFirstBootFlag();
  ReadWay2Leveling();
  ReadAutoBedGridData();
  if ((Manual_Leveling != 0xaa) && (Manual_Leveling != 0x55))
  {
    Manual_Leveling = 0xaa;
    SaveWay2Leveling();
  }
  SERIAL_ECHOPAIR("MEANL_L:", Manual_Leveling);
  zprobe_zoffset = Z_PROBE_OFFSET_FROM_EXTRUDER;
  if (FirstBootFlag != 0xa5)
  {
    FirstBootFlag = 0xa5;
    SaveFirstBootFlag();
    for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
    {
      for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
        z_values[x][y] = -3.5;
    };
    //  Manual_Leveling=0xaa;
    bilinear_grid_spacing[0] = int((RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (GRID_MAX_POINTS_X - 1));
    bilinear_grid_spacing[1] = int((BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (GRID_MAX_POINTS_Y - 1));
    bilinear_start[0] = LEFT_PROBE_BED_POSITION;
    bilinear_start[1] = FRONT_PROBE_BED_POSITION;
    zprobe_zoffset = Z_PROBE_OFFSET_FROM_EXTRUDER;
    NEW_zprobe_zoffset = Z_PROBE_OFFSET_FROM_EXTRUDER;
    SaveAutoBedGridData();
    // SERIAL_ECHOPAIR("detection", NEW_zprobe_zoffset);
  }
  else
    zprobe_zoffset = NEW_zprobe_zoffset;
}
#endif
void SetUpFAN2_PIN()
{
  SET_OUTPUT(V5_COOLING_PIN);
  WRITE(V5_COOLING_PIN, LOW);
}
void Fan2Scan()
{
  if (thermalManager.degHotend(0) > 65)
    WRITE(V5_COOLING_PIN, HIGH);
  else
    WRITE(V5_COOLING_PIN, LOW);
}

void TFT_Commond_Scan()
{
#ifdef TFTmodel
  if (TFTbuflen < (TFTBUFSIZE - 1))
    get_command_from_TFT();
  if (TFTbuflen)
  {
    TFTbuflen = (TFTbuflen - 1);
    TFTbufindr = (TFTbufindr + 1) % TFTBUFSIZE;
  }
#endif
  static unsigned int Scancount = 0;
  //    static unsigned long timeoutToStatus = 0;
  if ((thermalManager.degHotend(0) < 5) || ((thermalManager.degHotend(0) > 280)))
    Scancount++;
  if (Scancount > 61000)
  {
    Scancount = 0;
    NEW_SERIAL_PROTOCOLPGM("J10");
    TFT_SERIAL_ENTER();
  } //T0 unnormal
}

void setupSDCARD()
{
  SET_INPUT(SD_DETECT_PIN);
  WRITE(SD_DETECT_PIN, HIGH);
  _delay_ms(300);
  card.initsd();
}

/*
void AssistLevelTest()
{
static unsigned char stepFlag=0;
static char i=0;
static float E_count=0;
if(stepFlag==0)
{
  enqueue_and_echo_commands_P(PSTR("G28"));
  enqueue_and_echo_commands_P(PSTR("M109 S200"));
  stepFlag=1;
}
else if((stepFlag<72)&&(stepFlag>0)&&HomeFlag)
{
  if(thermalManager.degTargetHotend(0)<197) return;
  feedrate_percentage =2000.0;
  stepFlag++;
  E_count+=0.99781;
  planner.buffer_line(TEST_GCODE[i][0],TEST_GCODE[i][1],0.2,E_count,feedrate_percentage/60,0);
  i++;
  if(i==35)i=0;
}
else if(stepFlag>72)
{
  current_position[X_AXIS]=20;
  current_position[Y_AXIS]=20;
  current_position[Z_AXIS]=10;
  planner.set_e_position_mm(0);
//  planner.synchronize();
  thermalManager.setTargetHotend(0,0); //EXTRADER 0 COOL DOWN
  stepFlag=0;
  HomeFlag=0;
  NEW_SERIAL_PROTOCOLPGM("J22");//level watching finish
  TFT_SERIAL_ENTER();
  enqueue_and_echo_commands_P(PSTR("G28"));
  AssistLeveTestflag=0;
  i=0;
  E_count=0;
}
}
*/
bool pauseCMDsendflag = false;
void pauseCMDsend()
{
  static char temp = 0;
  if (commands_in_queue < BUFSIZE)
  {
    temp++;
    if (temp == 1)
      enqueue_and_echo_commands_P(PSTR("G91"));
    if (temp == 2)
    {
      enqueue_and_echo_commands_P(PSTR("G1 Z+20"));
      pauseCMDsendflag = false;
      temp = 0;
    }
  }
}
/*
void MY_AUTOlevelAlarm()
{
  if((READ(Z_TEST)==0))  tone(BEEPER_PIN, 4000,1);
  else noTone(BEEPER_PIN);
}
*/

void SDCARD_UPDATA()
{

  bool sd_status = IS_SD_INSERTED();
  if (sd_status != lcd_sd_status)
  {
    if (sd_status)
    {
      card.initsd();
#ifdef TFTmodel
      MyGetFileNr();
      NEW_SERIAL_PROTOCOLPGM("J00");
      TFT_SERIAL_ENTER();
#endif
    }
    else
    {
      card.release();
#ifdef TFTmodel
      NEW_SERIAL_PROTOCOLPGM("J01");
      TFT_SERIAL_ENTER();
#endif
    }
    lcd_sd_status = sd_status;
  }
}

#if ENABLED(EEPROM_SETTINGS)
/* Declared in configuration_store.cpp */
extern void _EEPROM_writeData(int &pos, uint8_t *value, uint8_t size);
extern void _EEPROM_readData(int &pos, uint8_t *value, uint8_t size);

//Should be same as in configuration_store.cpp
#define EEPROM_VERSION "V27"

#define DUMMY_PID_VALUE 3000.0f
#define EEPROM_START() int eeprom_index = EEPROM_OFFSET
#define EEPROM_SKIP(VAR) eeprom_index += sizeof(VAR)
#define EEPROM_WRITE(VAR) _EEPROM_writeData(eeprom_index, (uint8_t *)&VAR, sizeof(VAR))
#define EEPROM_READ(VAR) _EEPROM_readData(eeprom_index, (uint8_t *)&VAR, sizeof(VAR))

#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t *)&value, sizeof(value))
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t *)&value, sizeof(value))

#ifdef OutageTest
float last_position[4] = {0.0, 0.0, 0.0, 0.0};
long last_sd_position[1] = {0};
//extern float MYfeedrate_mm_s;
void OutageSave()
{
  char ver[4] = "000";
  int j = 20;
  EEPROM_WRITE_VAR(j, ver);
  // last_sd_position[0]=card.GetLastSDpos();
  last_position[0] = current_position[E_AXIS];
  last_position[1] = current_position[Z_AXIS];
  last_position[2] = current_position[Y_AXIS];
  last_position[3] = current_position[X_AXIS];

  EEPROM_WRITE_VAR(j, last_sd_position[0]);
  EEPROM_WRITE_VAR(j, last_position[0]); //E
  EEPROM_WRITE_VAR(j, last_position[1]); //Z
  EEPROM_WRITE_VAR(j, last_position[2]); //Y
  EEPROM_WRITE_VAR(j, last_position[3]); //X
  //  EEPROM_WRITE_VAR(j,MYfeedrate_mm_s);
}

void OutageRead()
{
  int i = 20;
  char stored_ver[4];
  char ver[4] = EEPROM_VERSION;
  EEPROM_READ_VAR(i, stored_ver);
  EEPROM_READ_VAR(i, last_sd_position[0]);
  EEPROM_READ_VAR(i, last_position[0]); //E
  EEPROM_READ_VAR(i, last_position[1]); //Z
  EEPROM_READ_VAR(i, last_position[2]); //Y
  EEPROM_READ_VAR(i, last_position[3]); //X

  /*
        SERIAL_ECHOPAIR(" MYx",last_position[3]);
        SERIAL_ECHOPAIR(" MYy",last_position[2]);
        SERIAL_ECHOPAIR(" MYz",last_position[1]);
        SERIAL_ECHOPAIR(" MYe",last_position[0]);
        SERIAL_ECHOPAIR(" SD",last_sd_position[0]);
      */
}

#endif

void SaveWay2Leveling()
{
  char ver[4] = "000";
  int j = 10;
  EEPROM_WRITE_VAR(j, ver);
  EEPROM_WRITE_VAR(j, Manual_Leveling);
}

void ReadWay2Leveling()
{
  int i = 10;
  char stored_ver[4];
  EEPROM_READ_VAR(i, stored_ver);
  EEPROM_READ_VAR(i, Manual_Leveling);
}

unsigned char FirstBootFlag;
#define bootEEPROM_OFFSET 85
void SaveFirstBootFlag()
{
  char ver[4] = "000";
  int j = bootEEPROM_OFFSET;
  FirstBootFlag = 0xa5;
  EEPROM_WRITE_VAR(j, ver);
  EEPROM_WRITE_VAR(j, FirstBootFlag);
}
void readFirstBootFlag()
{
  int i = bootEEPROM_OFFSET;
  char stored_ver[4];
  //   char ver[4]=EEPROM_VERSION;
  EEPROM_READ_VAR(i, stored_ver);
  EEPROM_READ_VAR(i, FirstBootFlag);
}

#define zEEPROM_OFFSET 50
//float last_z_offset[1]={0.0};
float last_z_offset = 0;
void SaveMyZoffset()
{
  char ver[4] = "000";
  int j = zEEPROM_OFFSET;
  last_z_offset = Current_z_offset;
  EEPROM_WRITE_VAR(j, ver);
  EEPROM_WRITE_VAR(j, last_z_offset);
}

void ReadMyZoffset()
{
  int i = zEEPROM_OFFSET;
  char stored_ver[4];
  char ver[4] = EEPROM_VERSION;
  EEPROM_READ_VAR(i, stored_ver);
  EEPROM_READ_VAR(i, last_z_offset);
}

#ifdef AUTO_BED_LEVELING_BILINEAR

#define sEEPROM_OFFSET 2000
void SaveAutoBedGridData()
{
  char ver[4] = "000";
  int j = sEEPROM_OFFSET;

  EEPROM_WRITE_VAR(j, ver);
  EEPROM_WRITE_VAR(j, bilinear_grid_spacing);
  EEPROM_WRITE_VAR(j, bilinear_start);
  EEPROM_WRITE_VAR(j, z_values);
  EEPROM_WRITE_VAR(j, NEW_zprobe_zoffset);

  //  EEPROM_WRITE_VAR(j,Manual_Leveling );
}
void ReadAutoBedGridData()
{
  int i = sEEPROM_OFFSET;
  char stored_ver[4];
  char ver[4] = EEPROM_VERSION;

  EEPROM_READ_VAR(i, stored_ver);
  EEPROM_READ_VAR(i, bilinear_grid_spacing);
  EEPROM_READ_VAR(i, bilinear_start);
  EEPROM_READ_VAR(i, z_values);
  EEPROM_READ_VAR(i, NEW_zprobe_zoffset);

  //    EEPROM_READ_VAR(i,Manual_Leveling);
  zprobe_zoffset = NEW_zprobe_zoffset;
}
#endif //AUTO_BED_LEVELING_BILINEAR
#endif //ENABLED(EEPROM_SETTINGS)

float Temperature::get_pid_output(const int8_t e)
{

//Define has been copied from temperature.cpp
#if defined(PID_K1) && !defined(PID_K2)
#define PID_K2 (1.0 - PID_K1)
#endif

#if HOTENDS == 1
  UNUSED(e);
#define _HOTEND_TEST true
#else
#define _HOTEND_TEST e == active_extruder
#endif
  float pid_output;
#if ENABLED(PIDTEMP)
#if DISABLED(PID_OPENLOOP)
  pid_error[HOTEND_INDEX] = target_temperature[HOTEND_INDEX] - current_temperature[HOTEND_INDEX];
  dTerm[HOTEND_INDEX] = PID_K2 * PID_PARAM(Kd, HOTEND_INDEX) * (current_temperature[HOTEND_INDEX] - temp_dState[HOTEND_INDEX]) + PID_K1 * dTerm[HOTEND_INDEX];
  temp_dState[HOTEND_INDEX] = current_temperature[HOTEND_INDEX];
#if HEATER_IDLE_HANDLER
  if (heater_idle_timeout_exceeded[HOTEND_INDEX])
  {
    pid_output = 0;
    pid_reset[HOTEND_INDEX] = true;
  }
  else
#endif
      if (pid_error[HOTEND_INDEX] > PID_FUNCTIONAL_RANGE)
  {
    pid_output = BANG_MAX;
    pid_reset[HOTEND_INDEX] = true;
  }
  else if (pid_error[HOTEND_INDEX] < -(PID_FUNCTIONAL_RANGE) || target_temperature[HOTEND_INDEX] == 0
#if HEATER_IDLE_HANDLER
           || heater_idle_timeout_exceeded[HOTEND_INDEX]
#endif
  )
  {
    pid_output = 0;
    pid_reset[HOTEND_INDEX] = true;
  }
  else
  {
    if (pid_reset[HOTEND_INDEX])
    {
      temp_iState[HOTEND_INDEX] = 0.0;
      pid_reset[HOTEND_INDEX] = false;
    }
    pTerm[HOTEND_INDEX] = PID_PARAM(Kp, HOTEND_INDEX) * pid_error[HOTEND_INDEX];
    temp_iState[HOTEND_INDEX] += pid_error[HOTEND_INDEX];
    iTerm[HOTEND_INDEX] = PID_PARAM(Ki, HOTEND_INDEX) * temp_iState[HOTEND_INDEX];

    pid_output = pTerm[HOTEND_INDEX] + iTerm[HOTEND_INDEX] - dTerm[HOTEND_INDEX];

#if ENABLED(PID_EXTRUSION_SCALING)
    cTerm[HOTEND_INDEX] = 0;
    if (_HOTEND_TEST)
    {
      long e_position = stepper.position(E_AXIS);
      if (e_position > last_e_position)
      {
        lpq[lpq_ptr] = e_position - last_e_position;
        last_e_position = e_position;
      }
      else
      {
        lpq[lpq_ptr] = 0;
      }
      if (++lpq_ptr >= lpq_len)
        lpq_ptr = 0;
      cTerm[HOTEND_INDEX] = (lpq[lpq_ptr] * planner.steps_to_mm[E_AXIS]) * PID_PARAM(Kc, HOTEND_INDEX);
      pid_output += cTerm[HOTEND_INDEX];
    }
#endif // PID_EXTRUSION_SCALING

    if (pid_output > PID_MAX)
    {
      if (pid_error[HOTEND_INDEX] > 0)
        temp_iState[HOTEND_INDEX] -= pid_error[HOTEND_INDEX]; // conditional un-integration
      pid_output = PID_MAX;
    }
    else if (pid_output < 0)
    {
      if (pid_error[HOTEND_INDEX] < 0)
        temp_iState[HOTEND_INDEX] -= pid_error[HOTEND_INDEX]; // conditional un-integration
      pid_output = 0;
    }
  }
#else
  pid_output = constrain(target_temperature[HOTEND_INDEX], 0, PID_MAX);
#endif // PID_OPENLOOP

#if ENABLED(PID_DEBUG)
  SERIAL_ECHO_START();
  SERIAL_ECHOPAIR(MSG_PID_DEBUG, HOTEND_INDEX);
  SERIAL_ECHOPAIR(MSG_PID_DEBUG_INPUT, current_temperature[HOTEND_INDEX]);
  SERIAL_ECHOPAIR(MSG_PID_DEBUG_OUTPUT, pid_output);
  SERIAL_ECHOPAIR(MSG_PID_DEBUG_PTERM, pTerm[HOTEND_INDEX]);
  SERIAL_ECHOPAIR(MSG_PID_DEBUG_ITERM, iTerm[HOTEND_INDEX]);
  SERIAL_ECHOPAIR(MSG_PID_DEBUG_DTERM, dTerm[HOTEND_INDEX]);
#if ENABLED(PID_EXTRUSION_SCALING)
  SERIAL_ECHOPAIR(MSG_PID_DEBUG_CTERM, cTerm[HOTEND_INDEX]);
#endif
  SERIAL_EOL();
#endif // PID_DEBUG

#else /* PID off */
#if HEATER_IDLE_HANDLER
  if (heater_idle_timeout_exceeded[HOTEND_INDEX])
    pid_output = 0;
  else
#endif
    pid_output = (current_temperature[HOTEND_INDEX] < target_temperature[HOTEND_INDEX]) ? PID_MAX : 0;
#endif

  return pid_output;
}

#if ENABLED(ULTRA_LCD)
void lcd_update()
{
#if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)
  bool sd_status = IS_SD_INSERTED();
  if (sd_status != lcd_sd_status && lcd_detected())
  {

    if (sd_status)
    {
      card.initsd();
      if (lcd_sd_status != 2)
        LCD_MESSAGEPGM(MSG_SD_INSERTED);
#ifdef TFTmodel
      MyGetFileNr();
      NEW_SERIAL_PROTOCOLPGM("J00");
      TFT_SERIAL_ENTER();
#endif
    }
    else
    {
      card.release();
      if (lcd_sd_status != 2)
        LCD_MESSAGEPGM(MSG_SD_REMOVED);
#ifdef TFTmodel
      NEW_SERIAL_PROTOCOLPGM("J01");
      TFT_SERIAL_ENTER();
#endif
    }
    lcd_sd_status = sd_status;
  }

#endif //SDSUPPORT && SD_DETECT_PIN
}
#endif // #if ENABLED(ULTRA_LCD)

#if ENABLED(SDSUPPORT)
void CardReader::Myls()
{
  lsAction = MySerial3Print;
  root.rewind();
  lsDive("", root);
}

uint16_t MyFileNrCnt = 0;
extern bool ReadMyfileNrFlag;
uint16_t fileoutputcnt = 0;
extern char *createFilename(char *buffer, const dir_t &p);

void CardReader::lsDive(const char *prepend, SdFile parent, const char *const match /*=NULL*/)
{
  dir_t p;
  uint8_t cnt = 0;
  while (parent.readDir(&p, longFilename) > 0)
  {
    if (DIR_IS_SUBDIR(&p) && lsAction != LS_Count && lsAction != LS_GetFilename) // hence LS_SerialPrint
    {

      char path[13 * 2];
      char lfilename[13];
      createFilename(lfilename, p);
      path[0] = 0;
      if (strlen(prepend) == 0) //avoid leading / if already in prepend
      {
        strcat(path, "/");
      }
      strcat(path, prepend);
      strcat(path, lfilename);
      strcat(path, "/");
      //Serial.print(path);

      SdFile dir;
      if (!dir.open(&parent, lfilename, O_READ))
      {
        if (lsAction == LS_SerialPrint)
        {
          SERIAL_ECHO_START();
          SERIAL_ECHOLN(MSG_SD_CANT_OPEN_SUBDIR);
          SERIAL_ECHOLN(lfilename);
          //          #ifdef TFTmodel
          //          NEW_SERIAL_ECHOLN(MSG_SD_CANT_OPEN_SUBDIR);
          //          NEW_SERIAL_ECHOLN(lfilename);
          //          #endif
        }
      }
      lsDive(path, dir);
      //close done automatically by destructor of SdFile
    }
    else
    {
      if (p.name[0] == DIR_NAME_FREE)
        break;
      if (p.name[0] == DIR_NAME_DELETED || p.name[0] == '.' || p.name[0] == '_')
        continue;
      if (longFilename[0] != '\0' &&
          (longFilename[0] == '.' || longFilename[0] == '_'))
        continue;
      if (p.name[0] == '.')
      {
        if (p.name[1] != '.')
          continue;
      }

      if (!DIR_IS_FILE_OR_SUBDIR(&p))
        continue;
      filenameIsDir = DIR_IS_SUBDIR(&p);
      if (!filenameIsDir)
      {
        if (p.name[8] != 'G')
          continue;
        if (p.name[9] == '~')
          continue;
      }
      //if(cnt++!=nr) continue;
      createFilename(filename, p);

      if (lsAction == MySerial3Print)
      {
        if (ReadMyfileNrFlag)
        {
          if ((strstr(filename, ".gco") != NULL) || (strstr(filename, ".GCO") != NULL))
            MyFileNrCnt++;
          //  MyFileNrCnt++;
        }
        else
        {
          //  if((MyFileNrCnt-filenumber*4)<4)
          if ((MyFileNrCnt - filenumber) < 4)
          {
            if (fileoutputcnt < (MyFileNrCnt - filenumber))
            {
              NEW_SERIAL_PROTOCOL(prepend);
              NEW_SERIAL_PROTOCOLLN(filename);
              //        NEW_SERIAL_PROTOCOL(prepend);
              NEW_SERIAL_PROTOCOLLN(longFilename);
            }
          }
          //    else if((fileoutputcnt>=((MyFileNrCnt-4)-filenumber*4))&&(fileoutputcnt<MyFileNrCnt-filenumber*4))
          else if ((fileoutputcnt >= ((MyFileNrCnt - 4) - filenumber)) && (fileoutputcnt < MyFileNrCnt - filenumber))
          {
            NEW_SERIAL_PROTOCOL(prepend);
            NEW_SERIAL_PROTOCOLLN(filename);
            //      NEW_SERIAL_PROTOCOL(prepend);
            NEW_SERIAL_PROTOCOLLN(longFilename);
          }
          fileoutputcnt++;
        }
        if (fileoutputcnt >= MyFileNrCnt)
          fileoutputcnt = 0;
      }
      else if (lsAction == LS_SerialPrint)
      {
        SERIAL_PROTOCOL(prepend);
        SERIAL_PROTOCOLLN(filename);
      }
      else if (lsAction == LS_Count)
      {
        nrFiles++;
      }
      else if (lsAction == LS_GetFilename)
      {
        //  if(cnt==nrFiles)
        //    return;
        //   cnt++;
        createFilename(filename, p);
        if (match != NULL)
        {
          if (strcasecmp(match, filename) == 0)
            return;
        }
        else if (cnt == nrFiles)
          return;
        cnt++;
      }
    }
  }
}

void CardReader::startFileprint()
{
  if (cardOK)
  {
    sdprinting = true;
    if (TFTresumingflag)
    {
      //      enquecommand_P(PSTR("G91"));
      enqueue_and_echo_commands_P(PSTR("G1 Z-20"));
      enqueue_and_echo_commands_P(PSTR("G90"));
      TFTresumingflag = false;
    }
  }
}

void CardReader::TFTStopPringing()
{
  sdprinting = false;
  TFTresumingflag = false;
  sdcardstartprintingflag = false;
  closefile();
  quickstop_stepper();
  NEW_SERIAL_PROTOCOLPGM("J16"); //STOP
  TFT_SERIAL_ENTER();
  //  autotempShutdown();
  disable_X();
  disable_Y();
  disable_Z();
  disable_e_steppers();
}

void CardReader::TFTgetStatus()
{
  //  if(TFTStatusFlag)
  //  {
  if (cardOK)
  {
    NEW_SERIAL_PROTOCOL(itostr3(percentDone()));
  }
  else
  {
    NEW_SERIAL_PROTOCOLPGM("J02");
    //        TFT_SERIAL_ENTER();
  }
  //    TFTStatusFlag=0;
  //  }
}
#endif // #if ENABLED(SDSUPPORT)

#endif // #ifdef VENDOR_CODE