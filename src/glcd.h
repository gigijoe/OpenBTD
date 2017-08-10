#ifndef _GLCD_H_
#define _GLCD_H_
#include "stm32f10x.h"

#define CONTROLLER_SPI_NUMBER      SPI2
#define CONTROLLER_SPI_PORT        GPIOB
#define CONTROLLER_SPI_SCK_PIN     GPIO_Pin_13
#define CONTROLLER_SPI_SCK_PINSRC  GPIO_PinSource13
#define CONTROLLER_SPI_MISO_PIN    GPIO_Pin_14
#define CONTROLLER_SPI_MISO_PINSRC GPIO_PinSource14
#define CONTROLLER_SPI_MOSI_PIN    GPIO_Pin_15
#define CONTROLLER_SPI_MOSI_PINSRC GPIO_PinSource15

#define CONTROLLER_SPI_DC_PORT  GPIOB
#define CONTROLLER_SPI_DC_PIN   GPIO_Pin_12
#define CONTROLLER_SPI_RST_PORT GPIOA
#define CONTROLLER_SPI_RST_PIN  GPIO_Pin_1

void Glcd_Init(uint8_t contrast, uint8_t bias);
void Glcd_Display(void);

#endif
