// Library for DS18B20
// Ver_1.0

/*========== LIBRARY DESCRIPTION ==========
- Library use STM32F3xx_HAL_Driver 
*/

#ifndef ds18b20_H_
#define ds18b20_H_
#include "stm32f1xx_hal.h"

#define SEARCH_ROM      0xF0
#define READ_ROM        0x33
#define MATCH_ROM       0x55
#define SKIP_ROM        0xCC
#define ALRM_SEARCH     0xEC
// DS18B20 commands
#define START_CONV_T    0x44
#define WRITE_SCRPAD    0x4E
#define READ_SCRPAD     0xBE
#define COPY_SCRPAD     0x48
#define RECALL          0xB8
#define READ_PWR        0xB4

#define DS18B20_RESOLUTION 0.0625f


/*========== TYPEDEFS ==========*/

/**
  * @brief Struct for DS18B20 data
  * @ingroup ds18b20
  */
typedef struct {
  int16_t hum;
  int16_t tmpr;
  uint8_t paritet;
  uint8_t error;
} ds18b20_data_t;

//========== VARIABLES ==========

//========== FUNCTIONS PROTOTYPES ==========

int ds18b20_init (void);
void ds18b20_deinit(void);
ds18b20_data_t ds18b20_get (uint8_t channel);
void ds18b20_task (void const * argument);
uint8_t crc8(uint8_t *pcBlock, int len);

#endif /* ds18b20_H_ */
