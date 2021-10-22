
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

  sConfigOC.Pulse = 65530;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /* Infinite loop */
  uint8_t led_level = 0;
  uint8_t state = 0; // STAND-BY
  uint32_t i = 0;

  for(;;)
  {
	switch (state){
	  // LED Blinking - Stand-by
	  case (0):
		led_level++;

	    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	    if (led_level > 127)
	    {
	      sConfigOC.Pulse = (255 - led_level) * 368;
	    }
	    else
	    {
	      sConfigOC.Pulse = led_level * 368;
	    }
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

		if (HAL_GPIO_ReadPin(PON_GPIO_Port, PON_Pin) == GPIO_PIN_RESET)
		{
			osDelay(1000);
            state = 1;
			led_level = 0;

			// Wait for button release
			for (i = 0 ; i < 100 ; i++)
			{
			  osDelay(100);
			  if (HAL_GPIO_ReadPin(PON_GPIO_Port, PON_Pin) == GPIO_PIN_SET) break;
			}
		}
	    break;

	  // ON requested
	  case (1):
		state = 0;
	    // search zero crossing
	    for (i = 0 ; i < 100 ; i++)
	    {
     		if (HAL_GPIO_ReadPin(AC_IN_GPIO_Port, AC_IN_Pin) == GPIO_PIN_SET)
	   		{
     		    for (i = 0 ; i < 100000 ; i++)
     		    {
     	     		if (HAL_GPIO_ReadPin(AC_IN_GPIO_Port, AC_IN_Pin) == GPIO_PIN_RESET)
     		   		{
     	     			// Transition from high to low - close to zero crossing
     	     			HAL_GPIO_WritePin(AC_CTRL_GPIO_Port, AC_CTRL_Pin, GPIO_PIN_SET);
     	     			state = 2;

     	     		    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
     	     		    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    	     		    sConfigOC.Pulse = 65535;
     	     			HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
     	     			HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
     	     			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
     	     			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
     	     			break;
     		   		}
     		    }
     		    if ((i == 100000) || (state = 2)) break;
            }
	    	osDelay(10);
	    }
		break;

	  // ON
	  case (2):
	    // Check for off request
		if (HAL_GPIO_ReadPin(PON_GPIO_Port, PON_Pin) == GPIO_PIN_RESET)
		{
		  state = 3;
		}

	    // Monitor system while is ON

		break;

	  // OFF requested
	  case (3):
  	    state = 0;
		// Switch off transformer
		HAL_GPIO_WritePin(AC_CTRL_GPIO_Port, AC_CTRL_Pin, GPIO_PIN_RESET);

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
