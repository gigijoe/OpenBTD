/*
 *  OpenBTD
 *
 *  Copyright (c) 2017, Steve Chang
 *  stevegigijoe@yahoo.com.tw
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "stm32f10x.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "bool.h"
#include "delay.h"
#include "usart.h"
#include "adc.h"
#include "ibus.h"

#ifdef PWM_FAN
#include "pwm.h"
#endif
#ifdef GLCD
#include "glcd.h"
#endif

/*
*
*/

static char *hextoa(uint8_t hex)
{
  static char a[3];
  uint8_t v = (hex & 0xf0) >> 4;
  switch(v) {
    case 10: a[0] = 'a';
      break;
    case 11: a[0] = 'b';
      break; 
    case 12: a[0] = 'c';
      break; 
    case 13: a[0] = 'd';
      break; 
    case 14: a[0] = 'e';
      break; 
    case 15: a[0] = 'f';
      break; 
    default: a[0] = (v + 0x30);
      break;
  }
  v = hex & 0xf;
  switch(v) {
    case 10: a[1] = 'a';
      break;
    case 11: a[1] = 'b';
      break; 
    case 12: a[1] = 'c';
      break; 
    case 13: a[1] = 'd';
      break; 
    case 14: a[1] = 'e';
      break; 
    case 15: a[1] = 'f';
      break; 
    default: a[1] = (v + 0x30);
      break;
  }
  a[2] = '\0';
  return a;
}

static char *hextodec(uint8_t hex)
{
  static char a[4];
  memset(a, 0x20, 4);
  if(hex >= 100) {
    a[0] = hex / 100 + 0x30;
    a[1] = (hex % 100) / 10 + 0x30;
    a[2] = (hex % 10) + 0x30;
    a[3] = '\0';
  } else if(hex >= 10) {
    a[0] = hex / 10 + 0x30;
    a[1] = hex % 10 + 0x30;
  } else
    a[0] = hex + 0x30;
  a[3] = '\0';
  return a;
}

static uint8_t atohex(char *s)
{
  uint8_t value = 0;
  if(!s)
    return 0;

  if(*s >= '0' && *s <= '9')
    value = (*s - '0') << 4;
  else if(*s >= 'A' && *s <= 'F')
    value = ((*s - 'A') + 10) << 4;
  else if(*s >= 'a' && *s <= 'f')
    value = ((*s - 'a') + 10) << 4;

  s++;

  if(*s >= '0' && *s <= '9')
    value |= (*s - '0');
  else if(*s >= 'A' && *s <= 'F')
    value |= ((*s - 'A') + 10);
  else if(*s >= 'a' && *s <= 'f')
    value |= ((*s - 'a') + 10);

  return value;
}

#if 0
static char *ibus_device_name(uint8_t id) 
{
  switch(id) {
    case 0x00: return "GM Broadcast";
    case 0x08: return "SHD Sunroof Control";
    case 0x18: return "CDC CD-Player";
    case 0x24: return "HKM Tailgate lift";
    case 0x28: return "FUH Radio controlled clock";
    case 0x30: return "CCM Check control module";
    case 0x3b: return "NAV Navigation/Videomodule";
    case 0x3f: return "DIA Diagnostic";
    case 0x40: return "FBZV Remote control central locking";
    case 0x43: return "GTF Graphics driver for rear screen (in navigation system)";
    case 0x44: return "EWS Electronic immobiliser";
    case 0x46: return "CID Central information display";
    case 0x50: return "MFL Multi Functional Steering Wheel Buttons";
    case 0x51: return "MM0 Mirror memory";
    case 0x5b: return "IHK Integrated heating and air conditioning";
    case 0x60: return "PDC Park Distance Control";
    case 0x68: return "RAD Radio";
    case 0x6A: return "DSP Digital Sound Processor";
    case 0x70: return "RDC Tyre pressure control";
    case 0x72: return "SM0 Seat memory";
    case 0x73: return "SDRS Sirius Radio";
    case 0x76: return "CDCD CD changer, DIN size";
    case 0x7f: return "NAVE Navigation (Europe)";
    case 0x80: return "IKE Instrument Control Electronics";
    case 0x9b: return "MM1 Mirror memory";
    case 0x9c: return "MM2 Mirror memory";
    case 0xa0: return "FMID Rear multi-info-display";
    case 0xa4: return "ABM Air bag module";
    case 0xac: return "EHC Electronic height control";
    case 0xb0: return "SES Speed recognition system";
    case 0xbb: return "NAVJ Navigation (Japan)";
    case 0xbf: return "GLO Global, broadcast address";
    case 0xc0: return "MID Multi-Information Display Buttons";
    case 0xc8: return "TEL Telephone";
    case 0xd0: return "LCM Light control module";
    case 0xe0: return "IRIS Integrated radio information system";
    case 0xe7: return "ANZV OBC TextBar";
    case 0xed: return "TV Television";
    case 0xf0: return "BMBT Board Monitor Buttons";
    case 0xff: return "LOC local";
    default: return "Unknown";
  }
}
#endif

static char *ibus_device_alias(uint8_t id) 
{
  switch(id) {
    case 0x00: return "GM";
    case 0x08: return "SHD";
    case 0x18: return "CDC";
    case 0x24: return "HKM";
    case 0x28: return "FUH";
    case 0x30: return "CCM";
    case 0x3b: return "NAV";
    case 0x3f: return "DIA";
    case 0x40: return "FBZV";
    case 0x43: return "GTF";
    case 0x44: return "EWS";
    case 0x46: return "CID";
    case 0x50: return "MFL";
    case 0x51: return "MM0";
    case 0x5b: return "IHK";
    case 0x60: return "PDC";
    case 0x68: return "RAD";
    case 0x6A: return "DSP";
    case 0x70: return "RDC";
    case 0x72: return "SM0";
    case 0x73: return "SDRS";
    case 0x76: return "CDCD";
    case 0x7f: return "NAVE";
    case 0x80: return "IKE";
    case 0x9b: return "MM1";
    case 0x9c: return "MM2";
    case 0xa0: return "FMID";
    case 0xa4: return "ABM";
    case 0xac: return "EHC";
    case 0xb0: return "SES";
    case 0xbb: return "NAVJ";
    case 0xbf: return "GLO";
    case 0xc0: return "MID";
    case 0xc8: return "TEL";
    case 0xd0: return "LCM";
    case 0xe0: return "IRIS";
    case 0xe7: return "ANZV";
    case 0xed: return "TV";
    case 0xf0: return "BMBT";
    case 0xff: return "LOC";
    default: return "Unknown";
  }
}

#define GM 0x00 /*Body module*/
#define SHD 0x08 /*Sunroof Control*/
#define CDC 0x18 /*CD Changer*/
#define FUH 0x28 /*Radio controlled clock*/
#define CCM 0x30 /*Check control module*/
#define GT 0x3B /*Graphics driver (in navigation system)*/
#define DIA 0x3F /*Diagnostic*/
#define FBZV 0x40 /*Remote control central locking*/
#define GTF 0x43 /*Graphics driver for rear screen (in navigation system)*/
#define EWS 0x44 /*Immobiliser*/
#define CID 0x46 /*Central information display (flip-up LCD screen)*/
#define MFL 0x50 /*Multi function steering wheel*/
#define MM0 0x51 /*Mirror memory*/
#define IHK 0x5B /*Integrated heating and air conditioning*/
#define PDC 0x60 /*Park distance control*/
#define ONL 0x67 /*unknown*/
#define RAD 0x68 /*Radio*/
#define DSP 0x6A /*Digital signal processing audio amplifier*/
#define SM0 0x72 /*Seat memory*/
#define SDRS 0x73 /*Sirius Radio*/
#define CDCD 0x76 /*CD changer, DIN size.*/
#define NAVE 0x7F /*Navigation (Europe)*/
#define IKE 0x80 /*Instrument cluster electronics*/
#define MM1 0x9B /*Mirror memory*/
#define MM2 0x9C /*Mirror memory*/
#define FMID 0xA0 /*Rear multi-info-display*/
#define ABM 0xA4 /*Air bag module*/
#define KAM 0xA8 /*unknown*/
#define ASP 0xAC /*unknown*/
#define SES 0xB0 /*Speed recognition system*/
#define NAVJ 0xBB /*Navigation (Japan)*/
#define GLO 0xBF /*Global, broadcast address*/
#define MID 0xC0 /*Multi-info display*/
#define TEL 0xC8 /*Telephone*/
#define TCU 0xCA /*unknown (BMW Assist?)*/
#define LCM 0xD0 /*Light control module*/
#define GTHL 0xDA /*unknown*/
#define IRIS 0xE0 /*Integrated radio information system*/
#define ANZV 0xE7 /*Front display*/
#define RLS 0xE8 /*Rain/Light Sensor*/
#define TV 0xED /*Television*/
#define BMBT 0xF0 /*On-board monitor operating part*/
#define LOC 0xFF /*Local*/

static uint8_t XOR_Checksum(uint8_t *buf, uint16_t len)
{
  uint8_t checksum = 0;
  uint16_t i;
  for(i=0;i<len;i++)
    checksum = checksum ^ buf[i];
  return checksum;
}

#if 0

int IBus_Send(int argc, char *argv[])
{
  if(argc <= 4)
    return -1; /* Not enough data */
  uint8_t code[MAX_TX_LEN];
  code[0] = atohex(argv[1]); /* src */
  /* code[1] : The length of the packet whithout Source ID and length it-self. */
  code[2] = atohex(argv[2]); /* dest */
  int i, len = 1;
  for(i=3;i<argc;i++) {
    code[i] = atohex(argv[i]); /* data */
    len++;
  }
  len++; /* check sum length is 1 */
  code[1] = len; 
  code[i] = XOR_Checksum((uint8_t *)&code[0], len + 1);
#if 0
  Usart2_Puts("\r\n");
  Usart2_Printf("Source : %s\r\n", ibus_device_name(code[0]));
  Usart2_Printf("Length : 0x%x\r\n", code[1]);
  Usart2_Printf("Destination : %s\r\n", ibus_device_name(code[2]));
  Usart2_Printf("CRC : %s\r\n", hextoa(code[i])); /* All exclude CRC itself */
#endif
  Usart3_Write(&code[0], len + 2);

  return 0;
}

int IBus_SendRaw(int argc, char *argv[])
{
  if(argc <= 6)
    return -1; /* Not enough data */
  uint8_t code[MAX_TX_LEN];
  int i, len = 0;
  for(i=2;i<argc;i++)
    code[len++] = atohex(argv[i]); /* data */
#if 0
  Usart2_Puts("\r\n");
  Usart2_Printf("Source : %s\r\n", ibus_device_name(code[0]));
  Usart2_Printf("Length : 0x%x\r\n", code[1]);
  Usart2_Printf("Destination : %s\r\n", ibus_device_name(code[2]));
  Usart2_Printf("CRC : %s\r\n", hextoa(code[len-1])); /* All exclude CRC itself */
#endif
  Usart3_Write(&code[0], len);

  return 0;
}

#endif

int IBus_Send2(uint8_t src, uint8_t dest, uint8_t *data, uint8_t dataLen) 
{
  uint8_t code[MAX_TX_LEN];

  code[0] = src; /* src */
  /* code[1] : The length of the packet whithout Source ID and length it-self. */
  code[2] = dest; /* dest */
  int i, len = 1; /* dest length is 1 */
  for(i=0;i<dataLen;i++) {
    code[i+3] = data[i]; /* data */
    len++;
  }
  len++; /* check sum length is 1 */
  code[1] = len; 
  code[i+3] = XOR_Checksum((uint8_t *)&code[0], len + 1);
#if 0
  Usart2_Puts("\r\n");
  Usart2_Printf("Source : %s\r\n", ibus_device_name(code[0]));
  Usart2_Printf("Length : 0x%x\r\n", code[1]);
  Usart2_Printf("Destination : %s\r\n", ibus_device_name(code[2]));
  Usart2_Printf("CRC : %s\r\n", hextoa(code[i+3])); /* All exclude CRC itself */
#endif
  Usart3_Write(&code[0], len + 2);

  return 0;
}

int IBus_SendRaw2(uint8_t *raw, uint8_t len)
{
  Usart3_Write(&raw[0], len);

  return 0;
}

static uint8_t radioScreenCommand[MAX_RX_LEN] = {0};
static uint8_t radioScreenCommandSize = 0;

void IBus_RedrawRadioScreen(char *text)
{
#define MAX_RADIO_SCREEN_LENGTH 11
  uint8_t d[] = { 0x23, 0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x04 };

  if(text == 0 || strlen(text) <= 0) {
    IBus_Send2(0x68, 0xe7, d, 3+MAX_RADIO_SCREEN_LENGTH+1);
    return;
  }

  uint8_t len = strlen(text);

  if(len > MAX_RADIO_SCREEN_LENGTH)
    len = MAX_RADIO_SCREEN_LENGTH;

  memcpy(&d[3], text, len);
  IBus_Send2(0x68, 0xe7, d, 3+MAX_RADIO_SCREEN_LENGTH+1); /* Display on ANZV OBC TextBar */
}

static uint8_t bcScreenCommand[MAX_RX_LEN] = {0};
static uint8_t bcScreenCommandSize = 0;

void IBus_RedrawBcScreen(char *text)
{
#define MAX_BC_SCREEN_LENGTH 20

  if(text == 0 || strlen(text) <= 0)
    return;

  uint8_t len = strlen(text);

  if(len > MAX_BC_SCREEN_LENGTH)
    len = MAX_BC_SCREEN_LENGTH;

//  uint8_t d[] = { 0x23, 0x01, 0x20, 0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20 };
  uint8_t d[] = { 0x23, 0x01, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x04 };
  memcpy(&d[3], text, len);

  IBus_Send2(0x80, 0xe7, d, 3+MAX_BC_SCREEN_LENGTH+1); /* Display "12345678901234567890" on IKE - Text Screen (20) */
}

void IBus_RedrawIkeScreen(char *text)
{
#define MAX_IKE_SCREEN_LENGTH 20

  if(text == 0 || strlen(text) <= 0)
    return;

  uint8_t len = strlen(text);

  if(len > MAX_IKE_SCREEN_LENGTH)
    len = MAX_IKE_SCREEN_LENGTH;

  uint8_t d[] = { 0x23, 0x62, 0x30, 0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x04 };
  uint8_t *p = &d[4];
  uint8_t frontPorch = (MAX_IKE_SCREEN_LENGTH - len) >> 1;
  memcpy(p + frontPorch, text, len);

  IBus_Send2(0x68, 0x80, d, 4+MAX_IKE_SCREEN_LENGTH+1); /* Display "12345678901234567890" on IKE - Text Screen (20) */
}

/* SSM : System Status Monitor */

#define MAXIMUM_COOLANT_TEMPERATURE 105
#define DEFAULT_COOLANT_TEMPERATURE 95 /* target temperatur initialize to 95°C */
#define MININUM_COOLANT_TEMPERATURE 85

typedef enum { SSM_DISABLED, SSM_COOLANT_VOLTAGE, SSM_COOLANT_HEATER_CURRENT, SSM_SETUP_COOLANT_TEMPERATURE, SSM_HEATER_FORCE_ON, SSM_UNKNOWN } SsmMode;

typedef struct {
  SsmMode mode;
  bool refresh;
  bool heaterOn, heaterForceOn;
  bool radioOn;
  uint8_t temperture, targetTemperature;
  float voltage, heaterCurrent;
  uint8_t VolumeUpTick, VolumeDownTick;
} Ssm;

static Ssm ssm; 

void Ssm_Init()
{
  ssm.mode = SSM_COOLANT_VOLTAGE;
  ssm.refresh = true;
  ssm.heaterOn = false;
  ssm.heaterForceOn = false;
  ssm.radioOn = false;
  ssm.temperture = 0;
  ssm.targetTemperature = DEFAULT_COOLANT_TEMPERATURE;
  ssm.voltage = 0.0f;
  ssm.heaterCurrent = 0.0f;
  ssm.VolumeUpTick = 0;
  ssm.VolumeDownTick = 0;
}

void Ssm_HeaterOn()
{
  if(ssm.heaterOn == false)
    IBus_RedrawIkeScreen("Heater On");
  GPIO_SetBits(GPIOA, GPIO_Pin_1);  // turn on heater
  ssm.heaterOn = true;
}

void Ssm_HeaterOff(char *reason)
{
  if(ssm.heaterOn == true) {
    if(reason)
      IBus_RedrawIkeScreen(reason);
    else
      IBus_RedrawIkeScreen("Heater Off");
  }
  GPIO_ResetBits(GPIOA, GPIO_Pin_1);  // turn off heater
  ssm.heaterOn = false;  
}

void Ssm_Update()
{
  if(ssm.mode == SSM_DISABLED)
    return;

  float v, rv;

  v = (float) ADC_SLOT[1] / 4096 * 3.3 / 0.582;
  rv = floorf(v * 100.0f) / 100.0f; /* Round down to XX.XX */
//Usart2_Printf("Current is %f A\r\n", v);
  if((ssm.mode == SSM_COOLANT_HEATER_CURRENT || ssm.mode == SSM_HEATER_FORCE_ON) && 
      ssm.heaterCurrent != rv) {
    ssm.heaterCurrent = rv;
    ssm.refresh = true;
  }

  if(ssm.heaterCurrent >= 2.0f) { /* Should be around 1A. Over 2A may be short !!! */
    Ssm_HeaterOff("Heater Off : Over Current");
  }


  v = (float) ADC_SLOT[0] / 4096 * 3.3 * ((2.62 + 9.98) / 2.62);
  rv = floorf(v * 10.0f) / 10.0f; /* Round down to XX.X */
//Usart2_Printf("Voltage is %f V\r\n", v);  
  if(ssm.mode == SSM_COOLANT_VOLTAGE && 
      ssm.voltage != rv) {
    ssm.voltage = rv;
    ssm.refresh = true;
  }

  if(ssm.refresh == false)
    return;
#if 0
/* Thermostat Heater = TH */
  if(ssm.heaterOn == false && ssm.heaterCurrent > 0.01f) {
    IBus_RedrawIkeScreen("Heater : Error Short");
  }

  if(ssm.heaterOn == true) {
    if(ssm.heaterCurrent < 0.01)
      IBus_RedrawIkeScreen("Heater : Error Power");
    else if(ssm.heaterCurrent < 0.5)
      IBus_RedrawIkeScreen("Heater : Warning Fade");
  }
#endif
  uint8_t d[] = { 0x23, 0x00, 0x20, 0x58, 0x58, 0x58, 0x43, 0x20, 0x03, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x04 };
  memcpy(&d[3], hextodec(ssm.temperture), 3);

  if(ssm.mode == SSM_COOLANT_VOLTAGE) {
    char vs[5];
    snprintf(vs, 5, "%f", ssm.voltage);
    memcpy(&d[9], vs, 4);
    d[14] = 'V';
  } else if(ssm.mode == SSM_COOLANT_HEATER_CURRENT || ssm.mode == SSM_HEATER_FORCE_ON) {
    char vs[5];
    snprintf(vs, 5, "%f", ssm.heaterCurrent);
    memcpy(&d[9], vs, 4);
    d[14] = 'A';
  } else if(ssm.mode == SSM_SETUP_COOLANT_TEMPERATURE) {
    char vs[4];
    snprintf(vs, 4, "%d", ssm.targetTemperature);
    memcpy(&d[10], vs, 3);
    d[14] = 'C';    
  }

  IBus_Send2(0x68, 0xe7, d, 17); /* Display water temperture on ANZV OBC TextBar */

  ssm.refresh = false;
}

/*
*
*/

void IBus_DecodeIke(const uint8_t *p)
{
  static uint16_t speed = 0;
  static uint16_t rpm = 0;

  switch(p[3]) { /* Message ID */
    case 0x18: { /* Speed & RPM */
#if 0
/*
80 05 BF 18 ss rr cc 

ss = speed / 2 [km/h] (512km/h max)
rr = revs / 100 rpm
*/
      if(p[2] == GLO)
        Usart2_Printf("IKE --> GLO : Speed/RPM: Speed %d km/h, %d RPM\r\n", p[4] << 1, p[5]);
#endif
      speed = p[4] << 1;
      rpm = p[5] * 100;
    } break;
    case 0x19: { /* Temperature */
#if 0
      if(p[2] == GLO)
        Usart2_Printf("IKE --> GLO : Temperature, Outside %d°C, Coolant %d°C\r\n", p[4], p[5]);
#endif
      int8_t tt = ssm.targetTemperature;
      int8_t dt = 0; /* target temperature offset */

      int8_t ot = p[4]; /* outside temperature */
      if(ot < 10)
        dt = 0;
      else if(ot < 20)
        dt = 1;
      else if(ot < 30)
        dt = 2;
      else if(ot < 40)
        dt = 3;

      tt -= dt;

      if(rpm < 3000)
        dt = 0;
      else if(rpm < 4000)
        dt = 1;
      else if(rpm < 5000)
        dt = 2;
      else
        dt = 3;

      tt -= dt;
#if 0
      if(speed == 0)
        dt = 9;
      else if(speed < 60)
        dt = 6;
      else if(dt < 120)
        dt = 3;
      else
        dt = 0;

      tt -= dt;
#endif
      if(tt < MININUM_COOLANT_TEMPERATURE)
        tt = MININUM_COOLANT_TEMPERATURE;

      int8_t ct = p[5]; /* coolant temperature */
      if(rpm == 0 || /* Do NOT enable thermostat heater while engine stopped */
        (ct < tt && ssm.heaterForceOn == false)) 
        Ssm_HeaterOff(0);
      else
        Ssm_HeaterOn();

      if(ssm.temperture != ct) {
        ssm.temperture = ct;
        ssm.refresh = true;
      }
    } break;
  }
}
#if 0
const uint8_t BTN_NEXT_PRESSED[] = { 0x50, 0x04, 0x68, 0x3b, 0x01, 0x06 };
const uint8_t BTN_NEXT_RELEASED[] = { 0x50, 0x04, 0x68, 0x3b, 0x21, 0x26 };

const uint8_t BTN_PREV_PRESSED[] = { 0x50, 0x04, 0x68, 0x3b, 0x08, 0x0f };
const uint8_t BTN_PREV_RELEASED[] = { 0x50, 0x04, 0x68, 0x3b, 0x28, 0x2f };
#endif
const uint8_t BTN_VOLUME_UP[] = { 0x50, 0x04, 0x68, 0x32, 0x11, 0x1f };
const uint8_t BTN_VOLUME_DOWN[] = { 0x50, 0x04, 0x68, 0x32, 0x10, 0x1e };

const uint8_t BTN_RT_TELEPHONE[] = { 0x50, 0x04, 0xc8, 0x3b, 0x40, 0xe7 };

const uint8_t BTN_TELEPHONE_PRESSED[] = { 0x50, 0x04, 0xc8, 0x3b, 0x80, 0x27 };
const uint8_t BTN_TELEPHONE_RELEASED[] = { 0x50, 0x04, 0xc8, 0x3b, 0xa0, 0x07 };

void IBus_DecodeMfl(const uint8_t *p)
{ 
#if 0 
  if(memcmp(BTN_NEXT_PRESSED, p, 6) == 0) {
    //GPIO_ResetBits(GPIOB, GPIO_Pin_13); /* pull low */
  } else if(memcmp(BTN_NEXT_RELEASED, p, 6) == 0) {
    //GPIO_SetBits(GPIOB, GPIO_Pin_13); /* pull high */
  } else if(memcmp(BTN_PREV_PRESSED, p, 6) == 0) {
    //GPIO_ResetBits(GPIOB, GPIO_Pin_15); /* pull low */
  } else if(memcmp(BTN_PREV_RELEASED, p, 6) == 0) {
    //GPIO_SetBits(GPIOB, GPIO_Pin_15); /* pull high */
  } else 
#endif
  if(memcmp(BTN_VOLUME_UP, p, 6) == 0) {
    if(ssm.VolumeUpTick > 0)
      return;
    ssm.VolumeUpTick = 2;
    if(ssm.mode == SSM_SETUP_COOLANT_TEMPERATURE) {
      if(ssm.targetTemperature < MAXIMUM_COOLANT_TEMPERATURE) {
        ssm.targetTemperature++;
        ssm.refresh = true;
      }
    }
  } else if(memcmp(BTN_VOLUME_DOWN, p, 6) == 0) {
    if(ssm.VolumeDownTick > 0)
      return;
    ssm.VolumeDownTick = 2;
    if(ssm.mode == SSM_SETUP_COOLANT_TEMPERATURE) {
      if(ssm.targetTemperature > MININUM_COOLANT_TEMPERATURE) {
        ssm.targetTemperature--;
        ssm.refresh = true;
      }
    }
  } else if(memcmp(BTN_RT_TELEPHONE, p, 6) == 0) {
  } else if(memcmp(BTN_TELEPHONE_PRESSED, p, 6) == 0) {
  } else if(memcmp(BTN_TELEPHONE_RELEASED, p, 6) == 0) {
#if 0
    uint8_t d3[] = { 0x23, 0x62, 0x30, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30 };
    IBus_Send2(0x68, 0x80, d3, 25); /* Display "12345678901234567890" on IKE - Text Screen (20) */
#endif
#if 0
    uint8_t d[] = { 0x21, 0x40, 0x00, 0x09, 0x05, 0x05, 0x4D, 0x50, 0x33 };
    IBus_Send2(0x68, 0xe7, d, 9); /* Display "MP3" on ANZV OBC TextBar - Radio Screen */
#endif
#if 0
    uint8_t d2[] = { 0x23, 0x01, 0x20, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30 };
    IBus_Send2(0x80, 0xe7, d2, 23); /* Display "12345678901234567890" on ANZV OBC TextBar - BC Screen (20) */    
#endif
    if(ssm.mode == SSM_HEATER_FORCE_ON) {
      ssm.heaterForceOn = false;
      Ssm_HeaterOff(0); /* To turn again by IKE */      
    }

    if(++ssm.mode == SSM_UNKNOWN)
      ssm.mode = SSM_DISABLED;
    else
      ssm.refresh = true;

    if(ssm.mode == SSM_HEATER_FORCE_ON) {
      ssm.heaterForceOn = true;
      Ssm_HeaterOn();
    }

    if(ssm.mode == SSM_DISABLED) {
      //IBus_RedrawRadioScreen("");
      if(radioScreenCommandSize > 0)
        IBus_SendRaw2(radioScreenCommand, radioScreenCommandSize);
      if(bcScreenCommandSize > 0)
        IBus_SendRaw2(bcScreenCommand, bcScreenCommandSize);

      //IBus_RedrawBcScreen("BMW E38 Individual");
#if 0      
      uint8_t d1[] = { 0x23, 0x40, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20 };
      IBus_Send2(0x68, 0xe7, d1, 14); /* Display on ANZV OBC TextBar */
#endif      
    } else if(ssm.mode == SSM_SETUP_COOLANT_TEMPERATURE) {
      IBus_RedrawBcScreen("Setup Temperature +/-");
    } else if(ssm.mode == SSM_HEATER_FORCE_ON) {
      IBus_RedrawBcScreen("Force Heater On");
    } else {
      IBus_RedrawBcScreen("");
    }
    Ssm_Update();
  }
}

const uint8_t RADIO_POWER_OFF[] = { 0x68, 0x05, 0xe7, 0x23, 0x00, 0x20, 0x89 };

void IBus_DecodeRad(uint8_t *p)
{
  if(memcmp(RADIO_POWER_OFF, p, 6) == 0)
    ssm.radioOn = false;
  else if(p[2] == ANZV) {
    if((p[3] == 0x21 || p[3] == 0x23) && (p[4] == 0x40 || p[4] == 0xc0 || p[4] == 0x80))
      ssm.radioOn = true;
  }
//Usart2_Printf("\r\nRadio Power %s\r\n", ssm.radioOn ? "On" : "Off");
}

const uint8_t BTN_MID_TOKEN[] = { 0x31, 0x80, 0x00 };  

void IBus_DecodeMid(uint8_t *p)
{
  if(p[2] == RAD) {
    if(memcmp(&p[3], BTN_MID_TOKEN, 3) == 0) {
      if(p[6] & 0xf0) { /* Button released */
        switch(p[6] & 0x0f) {
          case 0: break;
          case 1: break;
          case 2: break;
          case 3: break;
          case 4: break;
          case 5: break;
          case 6: break;
          case 7: break;
          case 8: break;
          case 9: break;
          case 10: break;
          case 11: break;
          case 12: break;
        }
      } else { /* Button pressed */
//Usart2_Printf("\r\nRadio button %d pressed\r\n", p[6] & 0x0f);
        switch(p[6] & 0x0f) {
          case 0: break;
          case 1: break;
          case 2: break;
          case 3: break;
          case 4: break;
          case 5: break;
          case 6: break;
          case 7: break;
          case 8: break;
          case 9: break;
          case 10: break;
          case 11: break;
          case 12: break;
        }
      }
    }
  }
}

GPIO_InitTypeDef GPIO_InitStructure;

/*
*
*/

static uint32_t tim4Tick = 0;

void Tim4_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* NVIC_PriorityGroup */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  //基础设置，时基和比较输出设置，由于这里只需定时，所以不用OC比较输出
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
  
  TIM_DeInit(TIM4);

  TIM_TimeBaseStructure.TIM_Period = 1000;//装载值
  //prescaler is 72, that is 72000000/72/1000 = 1000Hz;
  TIM_TimeBaseStructure.TIM_Prescaler = 71;//分频系数
  //set clock division 
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
  //count up
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  //clear the TIM4 overflow interrupt flag
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  //TIM4 overflow interrupt enable
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  //enable TIM4
  TIM_Cmd(TIM4, DISABLE);
}

void Tim4_Enable(void)
{
  TIM_Cmd(TIM4, ENABLE);
}

static uint32_t tim4Tick_200ms = 0;

void Tim4_200ms(void)
{
  static bool ledSwitch = true;
  if(ledSwitch)
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);  // turn off all led
  else
    GPIO_SetBits(GPIOB, GPIO_Pin_12);  // turn on all led

  ledSwitch = !ledSwitch;

  if(ssm.VolumeUpTick > 0)
    ssm.VolumeUpTick--;
  if(ssm.VolumeDownTick > 0)
    ssm.VolumeDownTick--;  
}

static uint32_t tim4Tick_1000ms = 0;

void Tim4_1000ms(void)
{
  if(tim4Tick_1000ms == 2) { /* Show log ONLY after 2 secs of power on */
    //IBus_RedrawIkeScreen("BMW E38 Individual");
    IBus_RedrawBcScreen("BMW E38 OpenBTD");
  }

  Ssm_Update();
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
  
  delay_init(72); /* 72 MHz system clock */

  /* GPIOB Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  /* Configure PB12 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_SetBits(GPIOB, GPIO_Pin_12); /* pull high */
  GPIO_SetBits(GPIOB, GPIO_Pin_13); /* pull high */
  GPIO_SetBits(GPIOB, GPIO_Pin_14); /* pull high */
  GPIO_SetBits(GPIOB, GPIO_Pin_15); /* pull high */

  /* GPIOA and GPIOC clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOA, GPIO_Pin_1);

  ADC1_Init();

#ifdef PWM_FAN
  Pwm_Init();
  Pwm1_Reverse();
  //Pwm1_Pulse(10 * PwmPulseMax / 100);
  Pwm1_Pulse(0);
#else
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA, GPIO_Pin_0); /* pull high, in order to low down 12V PWM output */
#endif

#ifdef GLCD
  Glcd_Init(55, 0x04);
#endif
  Tim4_Init();
  Tim4_Enable();
#ifdef NAVCODER
  Usart2_Init(9600);
#else  
  Usart2_Init(115200);
  Usart2_Puts("\r\nOpenBTD v0.1");
  Usart2_Puts("\r\nAuthor : Steve Chang");
  Usart2_Puts("\r\n27th Auguest 2017");
#endif
  Usart3_Init(9600);

  Ssm_Init();
  uint32_t tick_200ms = tim4Tick_200ms;
  uint32_t tick_1000ms = tim4Tick_1000ms;

  /* To achieve GPIO toggling maximum frequency, the following  sequence is mandatory. 
     You can monitor PD0 or PD2 on the scope to measure the output signal. 
     If you need to fine tune this frequency, you can add more GPIO set/reset 
     cycles to minimize more the infinite loop timing.
     This code needs to be compiled with high speed optimization option.  */
  for(;;) {
#ifdef NAVCODER
    int len = Usart3_Poll();
    if(len > 0)
      Usart2_Write(Usart3_Gets(), len);

    len = Usart2_Poll();
    if(len > 0)
      Usart3_Write(Usart2_Gets(), len);
#else
    if(tick_200ms != tim4Tick_200ms) {
      tick_200ms = tim4Tick_200ms;
      Tim4_200ms();
    }

    if(tick_1000ms != tim4Tick_1000ms) {
      tick_1000ms = tim4Tick_1000ms;
      Tim4_1000ms();
    }

    int len = Usart2_Poll();
    if(len > 0) {
      uint8_t *p = (uint8_t *)Usart2_Gets();      
    }

    len = Usart3_Poll();

    if(len > 0) {
      char *p = Usart3_Gets();

      switch(p[0]) { /* src */
        case MFL: /* MFL Multi Functional Steering Wheel Buttons */
          IBus_DecodeMfl(&p[0]);
          break;
        case IKE: /* IKE Instrument Control Electronics */
          IBus_DecodeIke(&p[0]);
          break;
        case RAD: /* Radio */
          IBus_DecodeRad(&p[0]);
          break;
        case MID: /* Multi-info display */
          IBus_DecodeMid(&p[0]);
          break;
      }

      if(p[0] == RAD && p[2] == ANZV && 
          p[3] == 0x23) { /* Display title field */
        memcpy(radioScreenCommand, &p[0], len);
        radioScreenCommandSize = len;
      } else if(p[0] == IKE && p[2] == MID && 
          p[3] == 0x23 && (p[4] == 0x01 || p[4] == 0x04)) { /* Clock and BC field */
        memcpy(bcScreenCommand, &p[0], len);
        bcScreenCommandSize = len;
      } else if(p[0] == IKE && p[2] == ANZV &&
          p[3] == 0x24 && p[4] == 0x01) { /* Clock field */
        memcpy(bcScreenCommand, &p[0], len);
        bcScreenCommandSize = len;        
      }
    }
#endif
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

void TIM4_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
    tim4Tick++;
    if(tim4Tick % 200 == 0)
      tim4Tick_200ms++;
    if(tim4Tick % 1000 == 0)
      tim4Tick_1000ms++;

#ifdef USART2_LIN_BUS
    usart2_idle_tick++;
#endif
#ifdef USART3_LIN_BUS
    usart3_idle_tick++;
#endif
    TIM_ClearITPendingBit(TIM4, /*TIM_IT_Update*/ TIM_FLAG_Update);
  }
}
