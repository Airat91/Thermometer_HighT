#ifndef PIN_MAP_H
#define PIN_MAP_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#define PWR_PORT        GPIOA
#define PWR_PIN         GPIO_PIN_0
#define TMPR_PORT    	GPIOA
#define TMPR_PIN     	GPIO_PIN_1

#define DEBUG_TMS_PORT  GPIOA
#define DEBUG_TMS_PIN   GPIO_PIN_13
#define DEBUG_TCK_PORT  GPIOA
#define DEBUG_TCK_PIN   GPIO_PIN_14
#define MAX_CS_PORT     GPIOA
#define MAX_CS_PIN      GPIO_PIN_15

#define MAX_SCK_PORT    GPIOB
#define MAX_SCK_PIN     GPIO_PIN_3

#define MAX_MOSI_PORT   GPIOB
#define MAX_MOSI_PIN    GPIO_PIN_5

#define DATA_PORT       GPIOB
#define DATA_PIN        GPIO_PIN_14

#define LED_PORT        GPIOC
#define LED_PIN         GPIO_PIN_13

#endif // PIN_MAP_H
