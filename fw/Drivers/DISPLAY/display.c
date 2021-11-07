#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "tm_stm32f4_ili9341.h"
#include "tm_stm32f4_fonts.h"

extern osMessageQueueId_t displayQueueHandle;
extern const uint8_t state;

osEvent event;
#define BAT_1_X_OFFSET 16
#define BAT_2_X_OFFSET 122
#define BAT_3_X_OFFSET 229

#define X_PIX 320
#define Y_PIX 240

uint8_t voltage2percent(uint16_t voltage) {

	if (voltage < 11000) {
		return 0;
	}
	if (voltage > 12800) {
		return 100;
	}
	voltage -= 11000;

	/**
	 * 11V -> 0%
	 * 12.8V -> 100%
	 * range 0 - 1800
	 */
	return voltage / 18;
}

void displayTaskEntry(void const * argument)
{
	TM_ILI9341_Init();

	for(;;)	{
	   uint16_t data;

	   if (osMessageQueueGet (displayQueueHandle, &data, NULL, 0) == osOK) {
           if (data == 0) {  // Power-down
		        TM_ILI9341_Init();
		   } else if (data == 0x0001) { // Welcome screen
			    TM_ILI9341_Fill(ILI9341_COLOR_BLACK);
				TM_ILI9341_Puts((X_PIX/2) - (16 * 11 / 2), (Y_PIX/2) - (26 /2),"www.ioko.eu", &TM_Font_16x26, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
		   } else if (data < 0x0010) { // power-up progress dots
			   TM_ILI9341_DrawCircle(75 + (20 * (data - 1)), 150, 5, ILI9341_COLOR_WHITE);
			   TM_ILI9341_DrawCircle(75 + (20 * (data - 1)), 150, 4, ILI9341_COLOR_WHITE);
			   TM_ILI9341_DrawCircle(75 + (20 * (data - 1)), 150, 3, ILI9341_COLOR_WHITE);
		   } else if (data == 0x0011) { // power-up ok
			   TM_ILI9341_DrawImage(40, 0, 280, 240, 0);

			   TM_ILI9341_Puts((X_PIX/2) - 40, (Y_PIX/2) - 5,  "Fan:        RPM", &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_YELLOW3);
			   TM_ILI9341_Puts((X_PIX/2) - 40, (Y_PIX/2) + 5,  "Out:         ", &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_YELLOW3);
			   TM_ILI9341_Puts((X_PIX/2) - 10,  (Y_PIX/2) + 7,  "* * * * * *", &TM_Font_7x10, ILI9341_COLOR_GREEN2, ILI9341_COLOR_YELLOW3);

			   TM_ILI9341_DrawCircle((X_PIX/2) - 80, (Y_PIX/2) - 30, 5, ILI9341_COLOR_GREEN);
			   TM_ILI9341_DrawCircle((X_PIX/2) - 80, (Y_PIX/2) - 30, 4, ILI9341_COLOR_GREEN);
			   TM_ILI9341_DrawCircle((X_PIX/2) - 80, (Y_PIX/2) - 30, 3, ILI9341_COLOR_GREEN);
			   TM_ILI9341_DrawCircle((X_PIX/2) - 80, (Y_PIX/2) - 30, 2, ILI9341_COLOR_GREEN);

			   TM_ILI9341_DrawCircle((X_PIX/2) - 80, (Y_PIX/2) + 30, 5, ILI9341_COLOR_GREEN);
			   TM_ILI9341_DrawCircle((X_PIX/2) - 80, (Y_PIX/2) + 30, 4, ILI9341_COLOR_GREEN);
			   TM_ILI9341_DrawCircle((X_PIX/2) - 80, (Y_PIX/2) + 30, 3, ILI9341_COLOR_GREEN);
			   TM_ILI9341_DrawCircle((X_PIX/2) - 80, (Y_PIX/2) + 30, 2, ILI9341_COLOR_GREEN);

		   } else if (data < 0x0100) { // Reserved

		   } else if (data < 0x0200) { // Left VU

		   } else if (data < 0x0300) { // Right VU

		   } else if (data < 0x2000) { // Reserved

		   } else if ((data < 0x4000) && (state == ON)) { // Temp L
			   uint16_t temp = data & 0x0FFF;
			   uint8_t buffer[14];
//			   if (data & 0x1000) {
//				   TM_ILI9341_Puts((X_PIX/2) - 10, (Y_PIX/2) - 15, "-", &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_YELLOW3);
//			   }
			   snprintf(&buffer, 12, "Tem: %2.2d.%1.1d   C", temp/10, temp%10);
			   TM_ILI9341_Puts((X_PIX/2) - 40, (Y_PIX/2) - 15, buffer, &TM_Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_YELLOW3);
		   } else if (data < 0x6000) { // Temp R

		   } else if (data < 0x8000) { // RPM L

		   } else if (data < 0xA000) { // RPM R

		   } else if (data < 0xC000) { // Reserved

		   } else if (data < 0xE000) { // Reserved

		   }
	   }
   }
}
