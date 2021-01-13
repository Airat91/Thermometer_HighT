#include "stdint.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"

#ifndef MAX7219_H
#define MAX7219_H 1

/*========== TYPEDEFS ==========*/

typedef struct {
    char symb;
    u8 code;
}max7219_symbol_t;

/**
 * @brief max7219_spi
 * @ingroup max7219
 */
extern SPI_HandleTypeDef max7219_spi;
extern max7219_symbol_t max7219_font[];


/*========== FUNCTION PROTOTYPES ==========*/

int max7219_init (void);
void max7219_deinit (void);
int max7219_spi_init (void);
void max7219_spi_deinit (void);
void max7219_gpio_init (void);
void max7219_gpio_deinit (void);
void max7219_send(u8 addr, u8 data);
u8 max7219_get_symbol_code(char symb);
void max7219_print_string(char *string);
void max7219_clr(void);

#endif // MAX7219_H
