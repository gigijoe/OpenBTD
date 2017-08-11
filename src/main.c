/**
  ******************************************************************************
  * @file    GPIO/IOToggle/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <string.h>
#include <stdio.h>

#include "bool.h"
#include "delay.h"
#include "usart.h"
#include "adc.h"
#include "pwm.h"
#if 0
#include "glcd.h"
#endif
#include "ibus.h"

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

static uint8_t atohex8(char *s)
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

#define MAX_CMD_LEN 32

typedef struct {
  char data[MAX_CMD_LEN + 1];
  int len;
} Shell;

void Shell_Input(Shell *s, char c)
{
  if(s->len < (MAX_CMD_LEN - 1)) {
    if(c == 0x8) { /* backspace */
      s->data[--s->len] = '\0';
      return;
    } else if(c < 0x20)
      return;

    s->data[s->len++] = c;
    s->data[s->len] = '\0';
  }
}

char *Shell_InputString(Shell *s)
{
  return s->len > 0 ? s->data : 0;
}

void Shell_InputReset(Shell *s)
{
  memset(s->data, 0, MAX_CMD_LEN + 1);
  s->len = 0;
}

#define MAX_ID_COUNT 4
static uint8_t srcIdCount = 0;
static uint8_t srcId[MAX_ID_COUNT];
static uint8_t destIdCount = 0;
static uint8_t destId[MAX_ID_COUNT];
typedef enum {ibusStop, ibusRecv} IBusState;
static IBusState state = ibusStop;

int IBus_Help(void)
{
  Usart2_Puts("\r\nIBus Inspector\r\n");

  Usart2_Puts("\r\n src [dev id 0] [dev id 1] ... [dev id n]");
  Usart2_Puts("\r\n dest [dev id 0] [dev id 1] ... [dev id n]");
  Usart2_Puts("\r\n recv");
  Usart2_Puts("\r\n stop");

  //Usart2_Puts("\r\n send <src id> <dest id> <XX XX XX ...>");
  //Usart2_Puts("\r\n send raw <XX XX XX ...>");

  return 0;
}

int IBus_SetupSource(int argc, char *argv[])
{
  int i;
  srcIdCount = 0;
  memset(srcId, 0, MAX_ID_COUNT);
  for(i=1;i<argc;i++) {
    srcId[i-1] = atohex8(argv[i]);
    if(++srcIdCount >= MAX_ID_COUNT)
      break;
  }
  return 0;
}

int IBus_SetupDestination(int argc, char *argv[])
{
  int i;
  destIdCount = 0;
  memset(destId, 0, MAX_ID_COUNT);
  for(i=1;i<argc;i++) {
    destId[i-1] = atohex8(argv[i]);
    if(++destIdCount >= MAX_ID_COUNT)
      break;
  }
  return 0;
}

int IBus_StartReceive(int argc, char *argv[])
{
  state = ibusRecv;
  return 0;
}

int IBus_StopReceive(int argc, char *argv[])
{
  state = ibusStop;
  return 0;
}

int IBus_Send(int argc, char *argv[])
{
  if(argc <= 4)
    return -1; /* Not enough data */
  uint8_t code[MAX_TX_LEN];
  code[0] = atohex8(argv[1]); /* src */
  /* code[1] : The length of the packet whithout Source ID and length it-self. */
  code[2] = atohex8(argv[2]); /* dest */
  int i, len = 1;
  for(i=3;i<argc;i++) {
    code[i] = atohex8(argv[i]); /* data */
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

int IBus_SendRaw(int argc, char *argv[])
{
  if(argc <= 6)
    return -1; /* Not enough data */
  uint8_t code[MAX_TX_LEN];
  int i, len = 0;
  for(i=2;i<argc;i++)
    code[len++] = atohex8(argv[i]); /* data */
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

IBusState IBus_State() { return state; }

uint8_t IBus_ValidSource(uint8_t id)
{
  int i;
  if(srcIdCount == 0)
    return id;
  for(i=0;i<srcIdCount;i++)
    if(srcId[i] == id)
      return id;
  return 0;
}

uint8_t IBus_ValidDestination(uint8_t id)
{
  int i;
  if(destIdCount == 0)
    return id;
  for(i=0;i<destIdCount;i++)
    if(destId[i] == id)
      return id;
  return 0;
}

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

void IBus_RedrawBcScreen(char *text)
{
//#define MAX_BC_SCREEN_LENGTH 24
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

typedef struct {
  bool enable;
  bool radioPowerOn;
  char temperture[4];
} Ssm;

static Ssm ssm;

void Ssm_Init()
{
  ssm.enable = true;
  ssm.radioPowerOn = false;
  memset(ssm.temperture, 0x20, 4);
}

static char *BatteryVoltage(void)
{
  static char vs[5];
  //float v = (float) ADC_ConvertedValue / 4096 * 3.3 * (10.0 / 2.15);
  float v = (float) ADC_ConvertedValue / 4096 * 3.3 * (10.0 / 2.58);
  snprintf(vs, 5, "%f", v);

  return vs;
}

void Ssm_Update()
{
  if(ssm.enable == false)
    return;

  uint8_t d[] = { 0x23, 0x00, 0x20, 0x58, 0x58, 0x58, 0x43, 0x20, 0x03, 0x20, 0x20, 0x20, 0x20, 0x20, 0x56, 0x20, 0x04 };
  memcpy(&d[3], ssm.temperture, 3);

  char *v = BatteryVoltage();
  memcpy(&d[9], v, 4);

  IBus_Send2(0x68, 0xe7, d, 17); /* Display water temperture on ANZV OBC TextBar */
}

/*
*
*/

void IBus_DecodeIke(uint8_t *p)
{
  static uint16_t speed = 0;
  static uint16_t rpm = 0;
  static bool heater = false;

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
      int8_t tt = 94; /* target temperatur initialize to 85°C */
      int8_t dt = 0; /* target temperature offset */

      uint8_t ot = p[4]; /* outside temperature */
      if(ot < 10)
        dt = 0;
      else if(ot < 20)
        dt = 3;
      else if(ot < 30)
        dt = 6;
      else if(ot < 40)
        dt = 9;

      tt -= dt;

      if(rpm < 3000)
        dt = 0;
      else if(rpm < 5000)
        dt = 3;
      else
        dt = 6;

      tt -= dt;

      if(speed < 100)
        dt = 0;
      else if(dt < 200)
        dt = 3;
      else
        dt = 6;

      tt -= dt;

      if(tt < 85)
        tt = 85;

      uint8_t ct = p[5]; /* coolant temperature */
      if(ct < (uint8_t)tt) {
        if(heater == true)
          IBus_RedrawIkeScreen("Thermostat Off");
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);  // turn off heater
        heater = false;
      } else {
        if(heater == false)
          IBus_RedrawIkeScreen("Thermostat On");
        GPIO_SetBits(GPIOA, GPIO_Pin_1);  // turn on heater
        heater = true;
      }

      memcpy(ssm.temperture, hextodec(ct), 3);
    } break;
  }
}

const uint8_t BTN_NEXT_PRESSED[] = { 0x50, 0x04, 0x68, 0x3b, 0x01, 0x06 };
const uint8_t BTN_NEXT_RELEASED[] = { 0x50, 0x04, 0x68, 0x3b, 0x21, 0x26 };

const uint8_t BTN_PREV_PRESSED[] = { 0x50, 0x04, 0x68, 0x3b, 0x08, 0x0f };
const uint8_t BTN_PREV_RELEASED[] = { 0x50, 0x04, 0x68, 0x3b, 0x28, 0x2f };

const uint8_t BTN_VOLUME_UP[] = { 0x50, 0x04, 0x68, 0x32, 0x11, 0x1f };
const uint8_t BTN_VOLUME_DOWN[] = { 0x50, 0x04, 0x68, 0x32, 0x10, 0x1e };

const uint8_t BTN_RT_TELEPHONE[] = { 0x50, 0x04, 0xc8, 0x3b, 0x40, 0xe7 };

const uint8_t BTN_TELEPHONE_PRESSED[] = { 0x50, 0x04, 0xc8, 0x3b, 0x80, 0x27 };
const uint8_t BTN_TELEPHONE_RELEASED[] = { 0x50, 0x04, 0xc8, 0x3b, 0xa0, 0x07 };

void IBus_DecodeMfl(uint8_t *p)
{  
  if(memcmp(BTN_NEXT_PRESSED, p, 6) == 0) {
    GPIO_ResetBits(GPIOB, GPIO_Pin_13); /* pull low */
  } else if(memcmp(BTN_NEXT_RELEASED, p, 6) == 0) {
    GPIO_SetBits(GPIOB, GPIO_Pin_13); /* pull high */
  } else if(memcmp(BTN_PREV_PRESSED, p, 6) == 0) {
    GPIO_ResetBits(GPIOB, GPIO_Pin_15); /* pull low */
  } else if(memcmp(BTN_PREV_RELEASED, p, 6) == 0) {
    GPIO_SetBits(GPIOB, GPIO_Pin_15); /* pull high */
  } else if(memcmp(BTN_VOLUME_UP, p, 6) == 0) {
  } else if(memcmp(BTN_VOLUME_DOWN, p, 6) == 0) {
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
    ssm.enable = !ssm.enable;

    if(ssm.enable == false) {
      IBus_RedrawRadioScreen("");
      IBus_RedrawBcScreen("Monitor On");
#if 0      
      uint8_t d1[] = { 0x23, 0x40, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20 };
      IBus_Send2(0x68, 0xe7, d1, 14); /* Display on ANZV OBC TextBar */
#endif      
    } else {
      //IBus_RedrawIkeScreen("BMW E38 Individual");   
      IBus_RedrawBcScreen("Monitor Off");
      Ssm_Update();
    }
  }
}

const uint8_t RADIO_POWER_OFF[] = { 0x68, 0x05, 0xe7, 0x23, 0x00, 0x20, 0x89 };

void IBus_DecodeRad(uint8_t *p)
{
  if(memcmp(RADIO_POWER_OFF, p, 6) == 0)
    ssm.radioPowerOn = false;
  else if(p[2] == ANZV) {
    if((p[3] == 0x21 || p[3] == 0x23) && (p[4] == 0x40 || p[4] == 0xc0 || p[4] == 0x80))
      ssm.radioPowerOn = true;
  }

Usart2_Printf("\r\nRadio Power %s\r\n", ssm.radioPowerOn ? "On" : "Off");
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
Usart2_Printf("\r\nRadio button %d pressed\r\n", p[6] & 0x0f);
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

/*
*
*/

int Shell_Run(Shell *s)
{
  if(s->len == 0)
    return -1;

  int ret = -1;

  char *p = 0;
#define MAX_ARGC 16 
  char *argv[MAX_ARGC];
  uint16_t argc = 0;

  int i;
  for(i=(s->len-1);i>=0;i--) {
    if(s->data[i] <= 0x20 || s->data[i] > 0x7e) {
      s->data[i] = '\0'; /* Strip back */
      s->len--;
    } else
      break;
  }

  for(i=0;i<s->len;i++) {
    if(s->data[i] > 0x20 && s->data[i] <= 0x7e)
      break; 
    s->data[i] = '\0'; /* Strip front */
    s->len--;
  }

  p = &s->data[i];
  i = 0;
  while(i < s->len) {
    if(p[i] == 0x20) {
      p[i++] = '\0';
      continue;
    }
    argv[argc++] = &p[i++];
    while(p[i] > 0x20 && p[i] <= 0x7e)
      i++;
    p[i++] = '\0'; /* end string */
    if(argc >= MAX_ARGC)
      break;
  }
#if 0
  for(i=0;i<argc;i++) {
    Usart2_Printf("\r\nargv[%d] = %s", i, argv[i]); 
  }
#endif
  if(strcmp("help", argv[0]) == 0)
    ret = IBus_Help();
  else if(strcmp("src", argv[0]) == 0)
    ret = IBus_SetupSource(argc, argv);
  else if(strcmp("dest", argv[0]) == 0)
    ret = IBus_SetupDestination(argc, argv);
  else if(strcmp("recv", argv[0]) == 0)
    ret = IBus_StartReceive(argc, argv);
  else if(strcmp("stop", argv[0]) == 0)
    ret = IBus_StopReceive(argc, argv);
/*  
  else if(strcmp("send", argv[0]) == 0) {
    if(argc > 2 && strcmp("raw", argv[1]) == 0)
      ret = IBus_SendRaw(argc, argv);
    else
      ret = IBus_Send(argc, argv);
  }
*/
  Shell_InputReset(s);

  return ret;
}

static Shell shell;

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

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

static uint32_t tim4Tick_1ms = 0;

void Tim4_1ms(void)
{
}

static uint32_t tim4Tick_10ms = 0;

void Tim4_10ms(void)
{
}

static uint32_t tim4Tick_50ms = 0;

void Tim4_50ms(void)
{  
}

static uint32_t tim4Tick_100ms = 0;

void Tim4_100ms(void)
{
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
}

static uint32_t tim4Tick_1000ms = 0;

void Tim4_1000ms(void)
{
#if 0
  static bool ledSwitch = true;
  if(ledSwitch)
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);  // turn off all led
  else
    GPIO_SetBits(GPIOA, GPIO_Pin_1);  // turn on all led

  ledSwitch = !ledSwitch;
#endif
/*  
  uint8_t d2[] = { 0x3b, 0x01 };
  IBus_Send2(0x50, 0x68, d2, 2);
*/
  if(tim4Tick_1000ms == 2) { /* Show log ONLY after 2 secs of power on */
    //IBus_RedrawIkeScreen("BMW E38 Individual");
    IBus_RedrawBcScreen("BMW E38 Individual");
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

  Pwm_Init();
  Pwm1_Pulse(10 * PwmPulseMax / 100);
#if 0
  Glcd_Init(55, 0x04);
#endif
  Tim4_Init();
  Tim4_Enable();
#ifdef NAVCODER
  Usart2_Init(9600);
#else  
  Usart2_Init(115200);
  Usart2_Puts("\r\nIBus Inspector v0.0.2");
  Usart2_Puts("\r\nAuthor : Steve Chang");
  Usart2_Puts("\r\n26th October 2017");

  Usart2_Puts("\r\nIBus\\> ");
#endif
  Usart3_Init(9600);

  Ssm_Init();

  uint32_t tick_1ms = tim4Tick_1ms;
  uint32_t tick_10ms = tim4Tick_10ms;
  uint32_t tick_50ms = tim4Tick_50ms;
  uint32_t tick_100ms = tim4Tick_100ms;
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
    if(tick_1ms != tim4Tick_1ms) {
      tick_1ms = tim4Tick_1ms;
      Tim4_1ms();
    }

    if(tick_10ms != tim4Tick_10ms) {
      tick_10ms = tim4Tick_10ms;
      Tim4_10ms();
    }

    if(tick_50ms != tim4Tick_50ms) {
      tick_50ms = tim4Tick_50ms;
      Tim4_50ms();
    }

    if(tick_100ms != tim4Tick_100ms) {
      tick_100ms = tim4Tick_100ms;
      Tim4_100ms();
    }

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
#if 1      
      int i;
      for(i=0;i<len;i++) {
        if(p[i] == 0xd) { /* CR */
          char *sc = Shell_InputString(&shell);
          if(sc)
            if(Shell_Run(&shell) == -1)
              Usart2_Puts("\r\nIllegal command\r\nIBus\\> ");
            else
              Usart2_Puts("\r\nIBus\\> ");
          else
            Usart2_Puts("\r\nIBus\\> ");
        } else if((p[i] >= 0x20 && p[i] <= 0x7e) || p[i] == 0x8) {
          if(p[i] == 0x8) /*backspace */
            Usart2_Puts("\b \b"); /* backspace + space + backspace */
          else 
            Usart2_Write((uint8_t *)&p[i], 1);
          Shell_Input(&shell, p[i]);
        }
      }
#endif      
    }

    len = Usart3_Poll();
/*    
      if(len > 0) 
        Usart2_Write(Usart3_Gets(), len);
*/
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
#if 1
      if(IBus_State() == ibusStop)
        continue;

      if(p[0] != IBus_ValidSource(p[0]) ||
        p[2] != IBus_ValidDestination(p[2]))
        continue;

      Usart2_Puts("\r\n");

      int i;
      for(i=0;i<len;i++) {
        Usart2_Write((uint8_t *)hextoa(p[i]), 2);
        Usart2_Write((uint8_t *)" ", 1);
        if(i == 2 || i == (len - 2))
          Usart2_Write((uint8_t *)"| ", 2);
      }
      Usart2_Puts("\r\n");
      Usart2_Printf(" %s --> %s : ", ibus_device_alias(p[0]), ibus_device_alias(p[2]));
      for(i=0;i<len;i++) {
        if(p[i] < 0x20 || p[i] > 0x7e)
          Usart2_Write((uint8_t *)".", 1);
        else
          Usart2_Write((uint8_t *)&p[i], 1);
      }
      Usart2_Puts("\r\n");
#endif          
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
    tim4Tick_1ms++;
    if(tim4Tick % 10 == 0)
      tim4Tick_10ms++;
    if(tim4Tick % 50 == 0)
      tim4Tick_50ms++;
    if(tim4Tick % 100 == 0)
      tim4Tick_100ms++;
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
    //
    // 清除 TIM4
    TIM_ClearITPendingBit(TIM4, /*TIM_IT_Update*/ TIM_FLAG_Update);
  }
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
