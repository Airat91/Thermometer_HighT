#include "ds18b20.h"
#include "pin_map.h"
#include "main.h"
#include "cmsis_os.h"

/**
  * @defgroup ds18b20
  * @brief work with 1-wire temperature sensor DS18B20
  */

/*========== FUNCTIONS ==========*/

static void ds18b20_pin_input(void);
static void ds18b20_pin_output(void);
static void write_bit(u8 bit);
static void write_data(u8 data);
static u8 read_bit(void);
static u8 read_data(void);
static u8 reset_1_wire(void);

/**
 * @brief Init DS18B20 pin as input
 * @ingroup ds18b20
 */
static void ds18b20_pin_input(void){
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = DS18B20_PIN;
    HAL_GPIO_Init (DS18B20_PORT, &GPIO_InitStruct);
}
/**
 * @brief Init DS18B20 pin as output OD
 * @ingroup ds18b20
 */
static void ds18b20_pin_output(void){
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = DS18B20_PIN;
    HAL_GPIO_Init (DS18B20_PORT, &GPIO_InitStruct);
}


static void write_bit(u8 bit){
    ds18b20_pin_output();
    taskENTER_CRITICAL();
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    us_tim_delay(60 - bit * 59);
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
    taskEXIT_CRITICAL();
    ds18b20_pin_input();
    us_tim_delay(bit * 59);
    us_tim_delay(1);
}

static u8 read_bit(void){
    u8 result = 0;
    ds18b20_pin_output();
    taskENTER_CRITICAL();
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    us_tim_delay(1);
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
    ds18b20_pin_input();
    us_tim_delay(14);
    if(HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN)){
        result = 1;
    }
    taskEXIT_CRITICAL();
    us_tim_delay(46);
    return result;
}

static void write_data(u8 data){
    u8 temp = 0;
    for(u8 i = 0; i < 8; i++){
        temp = (data >> i)&0x01;
        write_bit(temp);
    }
}

static u8 read_data(void){
    u8 result = 0;
    for(u8 i = 0; i < 8; i++){
        if(read_bit()){
            result |= (0x01 << i);
        }
    }
    return result;
}

static u8 reset_1_wire(void){
    u8 result = 0;
    u32 timeout = 0;
    u32 start = 0;
    u32 presence = 0;

    taskENTER_CRITICAL();
    ds18b20_pin_output();
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    us_tim_delay(480);
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
    ds18b20_pin_input();
    start = us_tim_get_value();
    // wait presence pulse
    while((HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == 1)&&(timeout < 60)){
        timeout = us_tim_get_value() - start;
    }
    start = us_tim_get_value();
    while((HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == 0)&&(presence < 480)){
        presence = us_tim_get_value() - start;
    }
    taskEXIT_CRITICAL();
    if(presence < 240){
        result = 1;
    }
    us_tim_delay(480 - presence);
    return result;
}
/**
 * @brief Init DS8B20 by 1-wire
 * @return  0 - DS18B20 init successfull,\n
 *          -1 - error
 * @ingroup ds18b20
 */
int ds18b20_init (void) {
    int result = 0;
    u8 buff[9] = {0};

    if(reset_1_wire()){
        write_data(SKIP_ROM);
        write_data(WRITE_SCRPAD);
        write_data(0x00);
        write_data(0x00);
        write_data(0x7F);
        reset_1_wire();
        write_data(SKIP_ROM);
        write_data(READ_SCRPAD);
        for(u8 i = 0; i < 9; i++){
            buff[i] = read_data();
        }
        reset_1_wire();
        write_data(SKIP_ROM);
        write_data(COPY_SCRPAD);
        osDelay(100);
        result = 1;
    }
    return result;
}

uint8_t crc8(uint8_t *pcBlock, int len){
    uint8_t crc = 0xFF;
    int i;

    while (len--)
    {
        crc ^= *pcBlock++;

        for (i = 0; i < 8; i++)
            crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    }

    return crc;
}



/**
 * @brief ds18b20_task
 * @param argument
 */
void ds18b20_task (void const * argument){
    (void)argument;
    uint8_t buff[9] = {0};
    ds18b20_init();
    float t = 0.0f;
    u16 t_adc = 0;
    u8 valid = 0;

    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        reset_1_wire();
        write_data(SKIP_ROM);
        write_data(READ_SCRPAD);
        for(u8 i = 0; i < 9; i++){
            buff[i] = read_data();
        }
        if(crc8(buff,8) == buff[8]){
            valid = 1;
        }else{
            valid = 0;
        }
        t_adc = buff[0]+(u16)(buff[1]<<8);
        t = t_adc * DS18B20_RESOLUTION;
        reset_1_wire();
        write_data(SKIP_ROM);
        write_data(START_CONV_T);
        osDelayUntil(&last_wake_time, 1000);
    }
}
