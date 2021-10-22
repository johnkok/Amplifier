#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern SPI_HandleTypeDef hspi3;
extern osMessageQId displayQueueHandle;

uint32_t qdata;
uint8_t touch_pending = 0;

void touchTaskEntry(void const * argument) {
	uint8_t din[3];
	uint8_t dout[3];
	uint8_t pos[4];
	uint16_t x;
	uint16_t y;

	while(1) {
		touch_pending = 0;

		while (!touch_pending) {
		   osDelay(10);
		}

		/* 0x0B/0x06
		 *  v
		 *  ------------------
		 *  |                |
		 *  |                |
		 *  |                |
		 *  ------------------
		 *                   ^
		 *                  0x79/0x75
		 */

		/*A1 - Y*/
		dout[0] = 0x90;
		dout[1] = dout[2] = 0x00;
		HAL_SPI_TransmitReceive(&hspi3, dout, din, 3, 20);
		pos[0] = din[1];

		/*A5 - X*/
		dout[0] = 0xD0;
		HAL_SPI_TransmitReceive(&hspi3, dout, din, 3, 20);
		pos[1] = din[1];

		/*A3 - Z1*/
		dout[0] = 0xB0;
		HAL_SPI_TransmitReceive(&hspi3, dout, din, 3, 20);
		pos[2] = din[1];

		/*A4 - Z2*/
		dout[0] = 0xC0;
		HAL_SPI_TransmitReceive(&hspi3, dout, din, 3, 20);
		pos[3] = din[1];

		/* Normalize */
		if (pos[0] >= 0x0b && pos[1] >= 0x06 &&
			pos[0] <= 0x79 && pos[1] <= 0x75 &&
			pos[2] && pos[3]) { /* Bound check */
			y = (((uint32_t)pos[1] - 0x06) * 240) / (0x75 - 0x06);
			x = (((uint32_t)pos[0] - 0x0B) * 320) / (0x79 - 0x0B);
			qdata = x + (y << 16);
			//osMessagePut( displayQueueHandle, &qdata, 100);
			osDelay(50);
		}
	}
}
