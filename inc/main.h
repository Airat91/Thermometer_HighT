/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "type_def.h"

#define TIME_YIELD_THRESHOLD 100
#define MEAS_NUM 6
#define SAVED_PARAMS_SIZE 7
#define SKIN_NMB 6


/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

#ifdef __cplusplus
 extern "C" {
#endif


#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

typedef enum {
    TMPR = 0,
    TMPR_ADC,
    TMPR_V,
    VREFINT_ADC,
    AM2302_T,
    AM2302_H,
}dcts_meas_t;

typedef enum{
    MENU_NAVIGATION,
    DIGIT_POSITION,
    DIGIT_EDIT,
}navigation_t;

typedef enum{
    HIGH_T_AND_TIME = 0,
    HIGH_T_ONLY,
    TIME_ONLY,
    AM2302_T_AND_H,
    HIGH_T_AND_AM2302_T,
    HIGH_T_AND_AM2302_T_AND_TIME,
}skin_t;

typedef enum{
    VAL_UNKNOWN = 0,
    VAL_UINT8,
    VAL_INT8,
    VAL_UINT16,
    VAL_INT16,
    VAL_UINT32,
    VAL_INT32,
}edit_val_type;

typedef union{
    uint8_t * p_uint8;
    int8_t * p_int8;
    uint16_t * p_uint16;
    int16_t * p_int16;
    uint32_t * p_uint32;
    int32_t * p_int32;
}edit_val_p_type_t;

typedef union{
    uint8_t uint8;
    int8_t int8;
    uint16_t uint16;
    int16_t int16;
    uint32_t uint32;
    int32_t int32;
}edit_val_type_t;

typedef struct{
    edit_val_p_type_t p_val;
    edit_val_type_t val_min;
    edit_val_type_t val_max;
    uint8_t digit;
    uint8_t digit_max;
    edit_val_type type;
}edit_val_t;

typedef enum{
    DATA_PIN_DISABLE = 0,
    DATA_PIN_EXT_AM2302,
    DATA_PIN_CLONE_AM2302,
}data_pin_t;

typedef union{
    struct{
        uint16_t mdb_address;
        uint16_t mdb_bitrate;
        uint16_t light_lvl;
        uint16_t skin;
        uint16_t data_pin_config;
        uint16_t tmpr_coef_a;
        int16_t tmpr_coef_b;
    }params;
    uint16_t word[SAVED_PARAMS_SIZE];
}saved_to_flash_t;

typedef enum{
    IRQ_NONE = 0,
    IRQ_SEND_TMPR,
    IRQ_READ_RTC,
}irq_state_t;

void _Error_Handler(char *, int);
extern uint32_t us_cnt_H;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern IWDG_HandleTypeDef hiwdg;
extern osThreadId defaultTaskHandle;
extern osThreadId buttonsTaskHandle;
extern osThreadId displayTaskHandle;
extern osThreadId menuTaskHandle;
extern osThreadId controlTaskHandle;
extern osThreadId adcTaskHandle;
extern osThreadId am2302TaskHandle;
extern osThreadId navigationTaskHandle;
extern osThreadId uartTaskHandle;
extern uint8_t irq_state;
extern saved_to_flash_t config;

void display_task(void const * argument);
void am2302_task(void const * argument);
void rtc_task(void const * argument);
void navigation_task(void const * argument);
void uart_task(void const * argument);
void refresh_watchdog(void);
uint32_t uint32_pow(uint16_t x, uint8_t pow);
u16 str_smb_num(char* string, char symbol);

uint32_t us_tim_get_value(void);
void us_tim_delay(uint32_t us);


#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
