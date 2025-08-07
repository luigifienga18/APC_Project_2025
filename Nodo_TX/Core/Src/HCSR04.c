/*
 * HCR04.c
 *
 *  Created on: Jul 12, 2025
 *      Author: serenasavarese
 */
#include <HCSR04.h>

void delay_us(uint16_t us, TIM_HandleTypeDef * htim) {
    __HAL_TIM_SET_COUNTER(htim, 0);  // reset counter
    while (__HAL_TIM_GET_COUNTER(htim) < us);
}

void HCSR04_Trigger(GPIO_TypeDef * GPIO, uint16_t PIN, TIM_HandleTypeDef * htim) {
    HAL_GPIO_WritePin(GPIO, PIN, GPIO_PIN_SET);
    delay_us(10, htim);                                         // 10 us
    HAL_GPIO_WritePin(GPIO, PIN, GPIO_PIN_RESET);
}

