#pragma once

#include "stm32f1xx_hal.h"
#include "defines.h"

typedef struct{
   GPIO_TypeDef* port;
   uint16_t pin;
}pin_t;

extern volatile uint8_t matrix_button[];

void matrix_init();
void matrix_update();
