#include "pin_map.h"
#include "buttons.h"
#include "dcts.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal_gpio.h"


/**
  * @defgroup button
  * @brief work with buttons
  */

/**
  * @addtogroup button
  * @{
  */
#define BUTTONS_TASK_PERIOD 1
#define BUTTONS_NUM 2
/**
  * @}
  */

/**
 * @brief var for read buttons pressed time
 * @ingroup button
 */
button_t pressed_time[BUTTONS_NUM];

static void buttons_init(void);
/**
 * @brief Buttons state read task
 * @param argument - None
 * @ingroup button
 *
 * Increments button time while pressed and save it value after release
 */
void buttons_task (void const * argument){
    (void) argument;
    buttons_init();
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        for(u8 button = 0; button < BUTTONS_NUM; button++){
            if(!HAL_GPIO_ReadPin(pressed_time[button].port, pressed_time[button].pin)){
                if(pressed_time[button].last_state == BUTTON_RELEASE){
                    pressed_time[button].pressed = 0;
                }
                pressed_time[button].pressed += BUTTONS_TASK_PERIOD;
                pressed_time[button].last_state = BUTTON_PRESSED;
            }else{
                pressed_time[button].last_state = BUTTON_RELEASE;
                //pressed_time[button].pressed = 0;
            }
        }
        osDelayUntil(&last_wake_time, BUTTONS_TASK_PERIOD);
    }
}
/**
 * @brief Init buttons GPIOs and global pressed_time
 * @ingroup button
 */
static void buttons_init(void){
    /* pressed_time init */
    pressed_time[BUTTON_OK].pin     = BUTTON_1_PIN;
    pressed_time[BUTTON_OK].port    = BUTTON_1_PORT;
    pressed_time[BUTTON_BREAK].pin  = BUTTON_2_PIN;
    pressed_time[BUTTON_BREAK].port = BUTTON_2_PORT;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Buttons */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    for(u8 i = 0; i < BUTTONS_NUM; i++){
        GPIO_InitStruct.Pin = pressed_time[i].pin;
        HAL_GPIO_Init(pressed_time[i].port, &GPIO_InitStruct);
        pressed_time[i].pressed = 0;
        pressed_time[i].last_state = BUTTON_RELEASE;
    }
}
/**
 * @brief Checks how long button was pressed before last release
 * @param button - button from @ref button_list_t
 * @param time - pressed time in ms
 * @return  0 - button was pressed less than time,\n
 *          1 - button was pressed more than time
 * @ingroup button
 * @warning Resets pressed_time if return 1
 */
uint8_t button_click(button_list_t button, uint16_t time){
    uint8_t result = 0;
    if((pressed_time[button].pressed >= time)&&
            (HAL_GPIO_ReadPin(pressed_time[button].port, pressed_time[button].pin))){
        result = 1;
        pressed_time[button].pressed = 0;
    }
    return result;
}
/**
 * @brief Checks how long button is pressed now
 * @param button - button from @ref button_list_t
 * @param time - pressed time in ms
 * @return  0 - button is pressed less than time,\n
 *          1 - button is pressed more than time
 * @ingroup button
 */
uint8_t button_clamp(button_list_t button, uint16_t time){
    uint8_t result = 0;
    if((pressed_time[button].pressed >= time)&&
            (!HAL_GPIO_ReadPin(pressed_time[button].port, pressed_time[button].pin))){
        result = 1;
    }
    return result;
}
