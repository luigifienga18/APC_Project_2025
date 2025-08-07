/*
 * HCR04.h
 *
 *  Created on: Jul 12, 2025
 *      Author: serenasavarese
 */

#ifndef SRC_HCSR04_H_
#define SRC_HCSR04_H_

#include "stm32f3xx_hal.h"

typedef struct hcsr04_t {

	uint32_t ic_val1;
	uint32_t ic_val2;
	uint8_t is_first_capture;
	uint32_t difference;
	uint32_t distance_cm;

}hcsr04_t;


void HCSR04_Trigger(GPIO_TypeDef * GPIO, uint16_t PIN, TIM_HandleTypeDef * htim);
void delay_us(uint16_t us, TIM_HandleTypeDef * htim);



#endif /* SRC_HCR04_H_ */
