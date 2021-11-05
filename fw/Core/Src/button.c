/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

#define START_DELAY 700  // (sec * 100)

/* Externals ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern osMessageQueueId_t displayQueueHandle;
extern I2C_HandleTypeDef hi2c2;

uint8_t state = STAND_BY; // STAND-BY

/**
* @brief Function implementing the buttonTask thread.
* @param argument: Not used
* @retval None
*/
void startCodec(void)
{
	uint8_t buffer[20];
	uint8_t i,x;

	for (x = 0xFF ; x > 0 ; x--) {
	    for (i = 6 ; i <= 13 ; i++) {
		    buffer[i] = x;             // Channel level +0
		    HAL_I2C_Mem_Write(&hi2c2, CODEC_ADD, i, I2C_MEMADD_SIZE_8BIT, &buffer[i], 1, 10);
		    osDelay(10);
	    }
	}

}

void stopCodec(void)
{
	uint8_t buffer[17];
	uint8_t i;

	HAL_I2C_Mem_Read(&hi2c2, CODEC_ADD, 0, I2C_MEMADD_SIZE_8BIT, buffer, 17, 1000);
	buffer[0] = buffer[0] | 0x01; // PLL Power-Down
	buffer[2] = buffer[2] | 0x01; // DAC Power-Down
	buffer[4] = buffer[4] | 0x01; // mute master
	buffer[5] = 0xFF;             // Mute all channels
	for (i = 6 ; i <= 13 ; i++) {
		buffer[i] = 0xFF;             // Channel level -oo
	}
	HAL_I2C_Mem_Write(&hi2c2, CODEC_ADD, 0, I2C_MEMADD_SIZE_8BIT, buffer, 17, 1000);
}

void swOnOutputs(void)
{
  osDelay(200);
  HAL_GPIO_WritePin(CH1_OUT_GPIO_Port, CH1_OUT_Pin, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(CH2_OUT_GPIO_Port, CH2_OUT_Pin, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(CH3_OUT_GPIO_Port, CH3_OUT_Pin, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(CH4_OUT_GPIO_Port, CH4_OUT_Pin, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(CH5_OUT_GPIO_Port, CH5_OUT_Pin, GPIO_PIN_SET);
  osDelay(100);
  HAL_GPIO_WritePin(CH6_OUT_GPIO_Port, CH6_OUT_Pin, GPIO_PIN_SET);
  //startCodec();
}

void swOffOutputs(void)
{
  osDelay(100);
  HAL_GPIO_WritePin(CH1_OUT_GPIO_Port, CH1_OUT_Pin, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(CH2_OUT_GPIO_Port, CH2_OUT_Pin, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(CH3_OUT_GPIO_Port, CH3_OUT_Pin, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(CH4_OUT_GPIO_Port, CH4_OUT_Pin, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(CH5_OUT_GPIO_Port, CH5_OUT_Pin, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(CH6_OUT_GPIO_Port, CH6_OUT_Pin, GPIO_PIN_RESET);
//  stopCodec();
}

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
  uint32_t display_event;

  for(;;)
  {
	switch (state){
	  // LED Blinking - Stand-by
	  case (STAND_BY):
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
            state = ON_REQUEST;
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
	  case (ON_REQUEST):
	    // Main transformer will be switch on from the zero crossing interrupt based on state
		display_event = 0x0001;
		osMessageQueuePut(displayQueueHandle, &display_event, 10, 0);
	    delay_cnt = 0;
        state = SS_REQUEST;

     	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
     	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    	sConfigOC.Pulse = 65535;
     	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
     	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
     	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
     	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
     	break;

	  case (SS_DELAY):
	    if (delay_cnt > START_DELAY) {
	      state++;
	      swOnOutputs();
		  display_event = 0x0011;
	      osMessageQueuePut(displayQueueHandle, &display_event, 10, 0);
		  display_event = 0x0200;
	      osMessageQueuePut(displayQueueHandle, &display_event, 10, 0);
		  display_event = 0x0300;
	      osMessageQueuePut(displayQueueHandle, &display_event, 10, 0);
	      break;
	    }
		if (HAL_GPIO_ReadPin(PON_GPIO_Port, PON_Pin) == GPIO_PIN_RESET)
		{
		  state = SB_REQUEST;
		  break;
		}
	    delay_cnt++;
	    if ((delay_cnt % 100) == 0) {
		  display_event = (0x0001 + (delay_cnt / 100));
	      osMessageQueuePut(displayQueueHandle, &display_event, 10, 0);
	    }
        break;

	  // ON
	  case (ON):
	    // Check for off request
		if (HAL_GPIO_ReadPin(PON_GPIO_Port, PON_Pin) == GPIO_PIN_RESET)
		{
		  state++;
		}

	    // TODO: Monitor system while is ON

		break;

	  // OFF requested
	  case (SB_PERFORM):
		swOffOutputs();
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

		display_event = 0x0000;
		osMessageQueuePut(displayQueueHandle, &display_event, 10, 0);

		state = STAND_BY;
		break;
	}
    osDelay(10);
  }
}
