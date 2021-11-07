#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"

extern osMessageQueueId_t displayQueueHandle;
extern osMessageQueueId_t fanQueueHandle;
extern TIM_HandleTypeDef htim5;

// Left sensor
#define	DS18B20_L_PORT 	TS1_GPIO_Port
#define	DS18B20_L_PIN 	TS1_Pin

// Right sensor
#define	DS18B20_R_PORT 	TS2_GPIO_Port
#define	DS18B20_R_PIN 	TS2_Pin

// Set GPIO as output
void set_gpio_output(uint32_t pin) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

// Set GPIO as input
void set_gpio_intput(uint32_t pin) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

// timer based dealy - in usec
void ds_delay (uint32_t us) {
    __HAL_TIM_SET_COUNTER(&htim5,0);
    while ((__HAL_TIM_GET_COUNTER(&htim5))<us);
}

uint8_t ds_crc8(const unsigned char * data, const uint32_t size)
{
    uint8_t crc = 0;
    for (uint32_t i = 0; i < size; ++i)
    {
        uint8_t inbyte = data[i];
        for (uint8_t j = 0; j < 8; ++j) {
        	uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}

// send start command
uint8_t DS18B20_Start(uint32_t pin) {
	uint8_t Response = 0;
	set_gpio_output(pin);
	HAL_GPIO_WritePin (GPIOD, pin, 0);
	ds_delay(500);

	set_gpio_intput(pin);
	ds_delay(80);

	if (!(HAL_GPIO_ReadPin(GPIOD, pin))) Response = 1;
	else Response = -1;
	ds_delay(400);

	return Response;
}

// write to the device
void DS18B20_Write(uint32_t pin, uint8_t data) {
	set_gpio_output(pin);

	for (int i=0; i<8; i++)	{
		if ((data & (1<<i))!=0) {
			set_gpio_output(pin);
			HAL_GPIO_WritePin (GPIOD, pin, 0);
			ds_delay(1);
			set_gpio_intput(pin);
			ds_delay(50);
		} else {
			set_gpio_output(pin);
			HAL_GPIO_WritePin (GPIOD, pin, 0);
			ds_delay(50);
			set_gpio_intput(pin);
		}
	}
}

// read from the device
uint8_t DS18B20_Read(uint32_t pin) {
	uint8_t value=0;
	set_gpio_intput(pin);

	for (int i=0;i<8;i++) {
		set_gpio_output(pin);

		HAL_GPIO_WritePin (GPIOD, pin, 0);
		ds_delay(2);

		set_gpio_intput(pin);
		if (HAL_GPIO_ReadPin (GPIOD, pin)) {
			value |= 1<<i;
		}
		ds_delay(60);
	}
	return value;
}


void sensorATaskEntry(void const * argument) {
	uint16_t display_event;
	uint8_t buffer[9];
	int32_t i;

	for(;;) {
		HAL_TIM_Base_Start(&htim5);
		DS18B20_Start(DS18B20_L_PIN);
		HAL_Delay(1);
		DS18B20_Write(DS18B20_L_PIN, 0xCC);  // skip ROM
		DS18B20_Write(DS18B20_L_PIN, 0x44);  // convert t
		HAL_Delay (800);

		DS18B20_Start(DS18B20_L_PIN);
		HAL_Delay(1);
		DS18B20_Write(DS18B20_L_PIN, 0xCC);  // skip ROM
		DS18B20_Write(DS18B20_L_PIN, 0xBE);  // Read Scratch-pad

		for (i = 0; i < 9 ; i++) {
		    buffer[i] = DS18B20_Read(DS18B20_L_PIN);
		}

        //Check CRC
//		if (ds_crc8(&buffer, 7) == buffer[8]) {
			// send data
			display_event = 10*(((buffer[0] & 0xF0) >> 4) + ((buffer[1] & 0x07) << 4));
			display_event = display_event + (10 * (buffer[0] & 0x0F)) / 16;
			display_event |= 0x2000;
//			if (buffer[1] > 0x07) {
//				display_event |= 0x1000;
//			}
			osMessageQueuePut(displayQueueHandle, &display_event, 10, 0);
			osMessageQueuePut(fanQueueHandle, &display_event, 10, 0);

//		}

		osDelay(2000);
	}
}
