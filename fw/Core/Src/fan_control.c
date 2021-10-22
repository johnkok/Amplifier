#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim1;

void fanTaskEntry(void *argument)
{

  TIM_OC_InitTypeDef sConfigOC = {0};

  // Set FAN Low
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 670;
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
  for(;;)
  {
    //Read temperature A
	//Read temperature B
	//Set FAN A speed
	//Set Fan B speed

	osDelay(1000);

	// Check FAN A speed

	// Check FAN B speed

	osDelay(1000);
  }
}
