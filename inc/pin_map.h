#ifndef PIN_MAP_H
#define PIN_MAP_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#define PWR_PORT        GPIOA
#define PWR_PIN         GPIO_PIN_0
#define TMPR_PORT    	GPIOA
#define TMPR_PIN     	GPIO_PIN_1
    #define DS18B20_PORT    TMPR_PORT
    #define DS18B20_PIN     TMPR_PIN

#define RS_485_TX_PORT  GPIOA
#define RS_485_TX_PIN   GPIO_PIN_2
#define RS_485_RX_PORT  GPIOA
#define RS_485_RX_PIN   GPIO_PIN_3
#define RS_485_DE_PORT  GPIOA
#define RS_485_DE_PIN   GPIO_PIN_4

/*#define ST7735_RESET_PORT GPIOA
#define ST7735_RESET_PIN  GPIO_PIN_8
#define ST7735_CS_PORT  GPIOA
#define ST7735_CS_PIN   GPIO_PIN_9*/
#define BUTTON_1_PORT   GPIOA
#define BUTTON_1_PIN    GPIO_PIN_9

#define DEBUG_TMS_PORT  GPIOA
#define DEBUG_TMS_PIN   GPIO_PIN_13
#define DEBUG_TCK_PORT  GPIOA
#define DEBUG_TCK_PIN   GPIO_PIN_14


#define MAX_SCK_PORT    GPIOB
#define MAX_SCK_PIN     GPIO_PIN_3
#define MAX_CS_PORT     GPIOB
#define MAX_CS_PIN      GPIO_PIN_4
#define MAX_MOSI_PORT   GPIOB
#define MAX_MOSI_PIN    GPIO_PIN_5

#define BUTTON_2_PORT   GPIOB
#define BUTTON_2_PIN    GPIO_PIN_11
#define DATA_PORT       GPIOB
#define DATA_PIN        GPIO_PIN_12
/*#define ST7735_SCK_PORT GPIOB
#define ST7735_SCK_PIN  GPIO_PIN_13
#define ST7735_A0_PORT GPIOB
#define ST7735_A0_PIN  GPIO_PIN_14
#define ST7735_MOSI_PORT GPIOB
#define ST7735_MOSI_PIN  GPIO_PIN_15*/

#define LED_PORT        GPIOC
#define LED_PIN         GPIO_PIN_13

#endif // PIN_MAP_H
