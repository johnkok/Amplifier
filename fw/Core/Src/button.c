/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

#define START_DELAY 700  // (sec * 100)

/* Externals ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
uint8_t state = 0; // STAND-BY

/**
* @brief Function implementing the buttonTask thread.
* @param argument: Not used
* @retval None
*/

void buttonTaskEntry(void *argument)
{

  TIM_OC_InitTypeDef sConfigOC = {0};

  // Set LED blinking
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /* Infinite loop */
  uint8_t led_level = 0;
  uint32_t i = 0;
  uint32_t delay_cnt = 0;

  for(;;)
  {
	switch (state){
	  // LED Blinking - Stand-by
	  case (0):
		led_level++;

	    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	    if (led_level > 127)
	    {
	      sConfigOC.Pulse = (0XFF - led_level) * 0xFFFF/200;
	    }
	    else
	    {
	      sConfigOC.Pulse = led_level * 0xFFFF/200;
	    }
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

		if (HAL_GPIO_ReadPin(PON_GPIO_Port, PON_Pin) == GPIO_PIN_RESET)
		{
			osDelay(100);
            state = 1;
			led_level = 0;

			// Wait for button release
			for (i = 0 ; i < 200 ; i++)
			{
			  osDelay(100);
			  if (HAL_GPIO_ReadPin(PON_GPIO_Port, PON_Pin) == GPIO_PIN_SET) break;
			}
		}
	    break;

	  // ON requested
	  case (1):
	    // Main transformer will be switch on from the zero crossing interrupt based on state
	    delay_cnt = 0;
        state = 2;

     	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
     	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    	sConfigOC.Pulse = 65535;
     	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
     	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
     	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
     	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
     	break;

	  case (3):
	    if (delay_cnt > START_DELAY) {
	      state++;
	    }
		if (HAL_GPIO_ReadPin(PON_GPIO_Port, PON_Pin) == GPIO_PIN_RESET)
		{
		  state = 6;
		}
	    delay_cnt++;
        break;

	  // ON
	  case (5):
	    // Check for off request
		if (HAL_GPIO_ReadPin(PON_GPIO_Port, PON_Pin) == GPIO_PIN_RESET)
		{
		  state = 6;
		}

	    // TODO: Monitor system while is ON

		break;

	  // OFF requested
	  case (7):
  	    state = 0;
		// Switch off LED and backlight
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		sConfigOC.Pulse = 0;
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

		// Wait for button release
		for (i = 0 ; i < 100 ; i++)
		{
		  osDelay(100);
		  if (HAL_GPIO_ReadPin(PON_GPIO_Port, PON_Pin) == GPIO_PIN_SET) break;
		}

		break;
	}
    osDelay(10);
  }
}
