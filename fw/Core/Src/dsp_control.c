#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

void dspTaskEntry(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
}
