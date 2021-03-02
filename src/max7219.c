#include "max7219.h"
#include "pin_map.h"
#include "dcts.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal_rcc.h"
#include "main.h"
#include "string.h"

#define FONT_SYMB_NMB 60
#define DIGIT_NUM 8

/**
  * @defgroup max7219
  * @brief work with 8-digit display based on max7219 driver
  */

/*========= GLOBAL VARIABLES ==========*/

SPI_HandleTypeDef max7219_spi = {0};

/*========== FUNCTIONS ==========*/

/**
 * @brief Init max7219
 * @return  0 - max7219 init successfull,\n
 *          -1 - max7219 SPI init error
 * @ingroup max7219
 */
int max7219_init (void){
    int result = 0;
    max7219_gpio_init();

    if(max7219_spi_init()<0){
        result = -1;
    }
    max7219_send(0x0C,0x00);
    max7219_send(0x0C,0x01);
    max7219_send(0x09,0x00);
    max7219_send(0x0A,(u8)(config.params.light_lvl/10));    // light level
    max7219_send(0x0B,DIGIT_NUM - 1);
    //test
    /*max7219_send(0x0F,0x01);
    osDelay(500);
    max7219_send(0x0F,0x00);*/
    max7219_clr();


    return result;
}
/**
 * @brief Deinit max7219
 * @ingroup max7219
 */
void max7219_deinit (void){
    max7219_gpio_deinit();
    max7219_spi_deinit();
}
/**
 * @brief Init max7219 SPI
 * @return  0 - SPI inited successful'\n
 *          -1 - SPI init error
 * @ingroup max7219
 */
int max7219_spi_init (void){
    __HAL_RCC_SPI1_CLK_ENABLE();
    int result = 0;
    max7219_spi.Instance = SPI1;
    max7219_spi.Init.Mode = SPI_MODE_MASTER;
    max7219_spi.Init.Direction = SPI_DIRECTION_1LINE;
    max7219_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    max7219_spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    max7219_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    max7219_spi.Init.NSS = SPI_NSS_SOFT;
    max7219_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    max7219_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    max7219_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    max7219_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    max7219_spi.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&max7219_spi) != HAL_OK)
    {
        result = -1;
    }
    __HAL_SPI_ENABLE(&max7219_spi);
    SPI_1LINE_TX(&max7219_spi);
    return result;
}
/**
 * @brief Deinit max7219 SPI
 * @ingroup max7219
 */
void max7219_spi_deinit (void){
    HAL_SPI_DeInit(&max7219_spi);
}
/**
 * @brief Init max7219 gpio
 * @ingroup max7219
 */
void max7219_gpio_init (void){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(MAX_CS_PORT, MAX_CS_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = MAX_CS_PIN;
    HAL_GPIO_Init(MAX_CS_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin = MAX_SCK_PIN;
    HAL_GPIO_Init(MAX_SCK_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = MAX_MOSI_PIN;
    HAL_GPIO_Init(MAX_MOSI_PORT, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_SPI1_ENABLE();
}
/**
 * @brief Deinit max7219 gpio
 * @ingroup max7219
 */
void max7219_gpio_deinit (void){
    HAL_GPIO_DeInit(MAX_CS_PORT,MAX_CS_PIN);
    HAL_GPIO_DeInit(MAX_SCK_PORT,MAX_SCK_PIN);
    HAL_GPIO_DeInit(MAX_MOSI_PORT,MAX_MOSI_PIN);
}
/**
 * @brief Send data to max7219
 * @param addr - address of max7219 register
 * @param data - data byte
 * @ingroup max7219
 */
void max7219_send(u8 addr, u8 data){
    u8 error = 0;
    u8 buff[2] = {addr, data};

    SPI_TypeDef *_lcd_spi = SPI1;
    max7219_spi.Instance = SPI1;

    HAL_GPIO_WritePin(MAX_CS_PORT, MAX_CS_PIN,GPIO_PIN_RESET);
    if(HAL_SPI_Transmit(&max7219_spi,&buff[0],2,100) == HAL_OK){
        while(((uint16_t)_lcd_spi->SR & SPI_SR_BSY)){
        }
    }else{
        error++;
    }
    HAL_GPIO_WritePin(MAX_CS_PORT, MAX_CS_PIN,GPIO_PIN_SET);
}

u8 max7219_get_symbol_code(char symb){
    u8 code = 0x08;
    u8 i = 0;
    while(i < FONT_SYMB_NMB){
        if(symb == max7219_font[i].symb){
            break;
        }
        i++;
    }
    if(i == FONT_SYMB_NMB){
        code = 0x08;
    }else{
        code = max7219_font[i].code;
    }
    return code;
}
void max7219_print_string(char *string){
    u8 len = 0;
    char * symb = string;
    u8 array[20] = {0};
    while(len < DIGIT_NUM){
        if(*symb == '\0'){
            break;
        }
        if(((*symb == '.')||(*symb == ','))&&(len > 0)){
            array[len - 1] |= 0x80;
            symb++;
        }else{
            array[len] = max7219_get_symbol_code(*symb);
            len++;
            symb++;
        }
    }
    len--;
    if(len != DIGIT_NUM - 1){
        len++;
        while(len < DIGIT_NUM){
            array[len] = max7219_get_symbol_code(' ');
            len++;
        }
    }
    for(len = 0; len < DIGIT_NUM; len++){
        max7219_send(DIGIT_NUM-len,array[len]);
    }
}

void max7219_clr(void){
    max7219_print_string("        ");
}

max7219_symbol_t max7219_font[FONT_SYMB_NMB] = {
    {.symb = ' ', .code = 0x00},
    {.symb = '0', .code = 0x7E},
    {.symb = '1', .code = 0x30},
    {.symb = '2', .code = 0x6D},
    {.symb = '3', .code = 0x79},
    {.symb = '4', .code = 0x33},
    {.symb = '5', .code = 0x5B},
    {.symb = '6', .code = 0x5F},
    {.symb = '7', .code = 0x70},
    {.symb = '8', .code = 0x7F},
    {.symb = '9', .code = 0x7B},
    {.symb = '-', .code = 0x01},
    {.symb = '_', .code = 0x08},
    {.symb = 'A', .code = 0x77},
    {.symb = 'a', .code = 0x77},
    {.symb = 'B', .code = 0x1F},
    {.symb = 'b', .code = 0x1F},
    {.symb = 'C', .code = 0x4E},
    {.symb = 'c', .code = 0x0D},
    {.symb = 'D', .code = 0x3D},
    {.symb = 'd', .code = 0x3D},
    {.symb = 'E', .code = 0x4F},
    {.symb = 'e', .code = 0x4F},
    {.symb = 'F', .code = 0x47},
    {.symb = 'f', .code = 0x47},
    {.symb = 'G', .code = 0x5E},
    {.symb = 'g', .code = 0x5E},
    {.symb = 'H', .code = 0x37},
    {.symb = 'h', .code = 0x17},
    {.symb = 'I', .code = 0x04},
    {.symb = 'i', .code = 0x04},
    {.symb = 'L', .code = 0x0E},
    {.symb = 'l', .code = 0x06},
    {.symb = 'N', .code = 0x15},
    {.symb = 'n', .code = 0x15},
    {.symb = 'O', .code = 0x7E},
    {.symb = 'o', .code = 0x1D},
    {.symb = 'P', .code = 0x67},
    {.symb = 'p', .code = 0x67},
    {.symb = 'R', .code = 0x05},
    {.symb = 'r', .code = 0x05},
    {.symb = 'S', .code = 0x5B},
    {.symb = 's', .code = 0x5B},
    {.symb = '*', .code = 0x63},//degree symbol
    {.symb = 'T', .code = 0x0F},
    {.symb = 't', .code = 0x0F},
    {.symb = 'U', .code = 0x3E},
    {.symb = 'u', .code = 0x1C},
    {.symb = 'Y', .code = 0x3B},
    {.symb = 'y', .code = 0x3B},
    {.symb = '(', .code = 0x4E},
    {.symb = '[', .code = 0x4E},
    {.symb = ')', .code = 0x78},
    {.symb = ']', .code = 0x78},
};
