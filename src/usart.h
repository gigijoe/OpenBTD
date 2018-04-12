/*
 * uart.h
 *
 *  Created on: 31 Oct 2016
 *      Author: Steve Chang
 */

#ifndef _USART_H
#define _USART_H

#include <stdint.h>

#define USART_TX_DMA 1

#define MAX_TX_LEN 32
#define MAX_RX_LEN 256 // this is the maximum string length of our string in characters

//#define USART1_ENABLE 1
//#define USART1_8E1 1
//#define USART1_SKIP_ECHO 1
//#define USART1_IBUS 1

void Usart1_Init(uint32_t baudrate);
void Usart1_Puts(char *string);
void Usart1_Printf(const char *fmt, ...);
void Usart1_Write(uint8_t *data, uint8_t len);
char *Usart1_Gets(void);
int Usart1_Read(uint8_t *data, uint8_t len);
int Usart1_Poll(void);
int Usart1_Poll2(uint16_t timeout_ms);
int Usart1_Poll3(int nBytes, uint16_t timeout_ms);

inline uint8_t *Usart1_TxBuf();
#ifdef USART1_IBUS
extern volatile uint32_t usart1_idle_tick;
#endif

#define USART2_ENABLE
//#define USART2_8E1 1
//#define USART2_SKIP_ECHO 1
//#define USART2_IBUS 1

void Usart2_Init(uint32_t baudrate);
void Usart2_Puts(char *string);
void Usart2_Printf(const char *fmt, ...);
void Usart2_Write(uint8_t *data, uint8_t len);
char *Usart2_Gets(void);
int Usart2_Read(uint8_t *data, uint8_t len);
int Usart2_Poll(void);
int Usart2_Poll2(uint16_t timeout_ms);
int Usart2_Poll3(int nBytes, uint16_t timeout_ms);

inline uint8_t *Usart2_TxBuf();
#ifdef USART2_IBUS
extern volatile uint32_t usart2_idle_tick;
#endif

#define USART3_ENABLE
#define USART3_8E1 1
#define USART3_SKIP_ECHO 1
#define USART3_IBUS 1

void Usart3_Init(uint32_t baudrate);
void Usart3_Puts(char *string);
void Usart3_Printf(const char *fmt, ...);
void Usart3_Write(uint8_t *data, uint8_t len);
char *Usart3_Gets(void);
int Usart3_Read(uint8_t *data, uint8_t len);
int Usart3_Poll(void);
int Usart3_Poll2(uint16_t timeout_ms);
int Usart3_Poll3(int nBytes, uint16_t timeout_ms);

inline uint8_t *Usart3_TxBuf();
#ifdef USART3_IBUS
extern volatile uint32_t usart3_idle_tick;
#endif

#endif
