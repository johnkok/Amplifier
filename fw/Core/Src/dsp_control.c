#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"

#define DSP_MP14_ADD 0xF5D0

extern I2C_HandleTypeDef hi2c1;

void dspTaskEntry(void *argument)
{
  uint8_t buffer[4];
  uint8_t i = 0;

  buffer[0] = 0xF5;
  buffer[1] = 0xD0;
  buffer[2] = 0;

  /* Infinite loop */
  for(;;)
  {

	// DSP heart-beat
    osDelay(50);
    buffer[2] = 0;
    HAL_I2C_Master_Transmit(&hi2c1,  i, &buffer[0], 4, 100);
    osDelay(50);
    buffer[3] = 1;
    HAL_I2C_Master_Transmit(&hi2c1,  i, &buffer[0], 4, 100);
    i = i + 2;
/*    osDelay(100);

    osDelay(100);
    buffer[2] = 0;
    HAL_I2C_Mem_Write(&hi2c1,  DSP_ADD, DSP_MP14_ADD, I2C_MEMADD_SIZE_16BIT, &buffer[2], 2, 100);
    osDelay(100);
    buffer[3] = 1;
    HAL_I2C_Mem_Write(&hi2c1,  DSP_ADD, DSP_MP14_ADD, I2C_MEMADD_SIZE_16BIT, &buffer[2], 2, 100);
    osDelay(100);
*/
  }
}
