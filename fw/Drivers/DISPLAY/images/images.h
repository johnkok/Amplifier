#ifndef IMAGES_IMAGES_H_
#define IMAGES_IMAGES_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

const uint8_t bat[];
const uint8_t bat_orange[];
const uint8_t bat_red[];
const uint8_t bat_green[];
const uint8_t therm[];
const uint8_t bon[];
const uint8_t boff[];
const uint8_t error[];

#define BAT 	0
#define BAT_R 	1
#define BAT_O 	2
#define BAT_G 	3
#define THERM 	4
#define BON 	5
#define BOFF 	6
#define ERR 	7

#endif /* IMAGES_IMAGES_H_ */
