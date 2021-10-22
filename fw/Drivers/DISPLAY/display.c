#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "images/images.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "tm_stm32f4_ili9341.h"
#include "tm_stm32f4_fonts.h"

extern osMessageQueueId_t displayQueueHandle;

//extern status_t status;

osEvent event;
#define BAT_1_X_OFFSET 16
#define BAT_2_X_OFFSET 122
#define BAT_3_X_OFFSET 229

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

void drawPoint(uint16_t X, uint16_t Y)
{

	/* Bridge pressed */
/*
	if (X > 80 && X < 130 &&
		Y > 45 && Y < 75) {
		if (status.bridge) {
			status.bridge = 0;
			TM_ILI9341_DrawImage(86, 50, 40, 20, BOFF);
		}
		else {
			status.bridge = 1;
			TM_ILI9341_DrawImage(86, 50, 40, 20, BON);
		}
	}
	else if ( 	X > 0 && X < 40 &&
				Y > 50 && Y < 85) {
		   TM_ILI9341_DrawFilledRectangle(0, 55, 36, 85, ILI9341_COLOR_BLACK);
	}
	else if ( 	X > 45 && X < 75 &&
				Y > 200 && Y < 240) {
		   TM_ILI9341_DrawFilledRectangle(50, 204, 86, 234, ILI9341_COLOR_BLACK);
	}
	else
		TM_ILI9341_DrawCircle(X, Y, 10, ILI9341_COLOR_BLUE);
*/
}


void displayTaskEntry(void const * argument)
{
    TM_ILI9341_Init();

    /* Cross */
    TM_ILI9341_DrawFilledRectangle(0, 120, 320, 121, ILI9341_COLOR_WHITE);
    TM_ILI9341_DrawFilledRectangle(106, 0, 107, 240*2, ILI9341_COLOR_WHITE);
    TM_ILI9341_DrawFilledRectangle(213, 0, 214, 240*2, ILI9341_COLOR_WHITE);

    /* Texts */
    TM_ILI9341_Puts(20,0,"Engine", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    TM_ILI9341_Puts(145,00,"Acc.", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    TM_ILI9341_Puts(245,00,"Solar", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);

    TM_ILI9341_Puts(20,122,"Engine", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    TM_ILI9341_Puts(122,122,"Exhaust", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    TM_ILI9341_Puts(235,122,"Status", &TM_Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);

    /* Batteries */
    TM_ILI9341_DrawImage(6, 88, 95, 30, BAT);
    TM_ILI9341_DrawImage(112, 88, 95, 30, BAT);
    TM_ILI9341_DrawImage(219, 88, 95, 30, BAT);

    /* Thermometers */
    TM_ILI9341_DrawImage(1, 145, 30, 90, THERM);
    TM_ILI9341_DrawImage(113, 145, 30, 90, THERM);
    TM_ILI9341_Puts(220,180,"Room temp:", &TM_Font_7x10, ILI9341_COLOR_YELLOW, ILI9341_COLOR_BLACK);
    TM_ILI9341_Puts(220,147,"Bridge:", &TM_Font_7x10, ILI9341_COLOR_YELLOW, ILI9341_COLOR_BLACK);

   for(;;)
   {
	   uint32_t data;

	   osMessageQueueGet (displayQueueHandle, &data, NULL, 0);

	   if (data){
           drawPoint(data & 0xFFFF, (data & 0xFFFF0000) >> 16);
	   }
	   else {

	   }
   }
}
