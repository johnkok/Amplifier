#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim1;
extern osMessageQueueId_t fanQueueHandle;

void fanTaskEntry(void *argument)
{

  TIM_OC_InitTypeDef sConfigOC = {0};

  // Set FAN Low
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 670*10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  /* Infinite loop */
  for(;;){
	  uint16_t data;

	  if (osMessageQueueGet (fanQueueHandle, &data, NULL, 0) == osOK) {
		  if (data < 0x2000) { // Reserved

		  } else if (data < 0x4000) { // Temp L
			  uint16_t temp = (data & 0x0FFF) / 10;
			  if (temp < 40) {
				  sConfigOC.Pulse = 670 * 5;
			  }else if (temp < 50) {
				  sConfigOC.Pulse = 670 * 6;
			  }else if (temp < 60) {
				  sConfigOC.Pulse = 670 * 7;
			  }else if (temp < 70) {
				  sConfigOC.Pulse = 670 * 8;
			  }else if (temp < 80) {
				  sConfigOC.Pulse = 670 * 9;
			  }else {
				  sConfigOC.Pulse = 669 * 10;
			  }

			  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
			  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

			  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
			  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		  } else if (data < 0x6000) { // Temp R

		  }
	  }

  }
}
