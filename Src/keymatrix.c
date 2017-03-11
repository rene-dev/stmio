#include "keymatrix.h"

const pin_t matrix_row[] = {
   {GPIOB, GPIO_PIN_11},
   {GPIOB, GPIO_PIN_10},
   {GPIOB, GPIO_PIN_2},
   {GPIOB, GPIO_PIN_1},
   {GPIOB, GPIO_PIN_0},
   {GPIOC, GPIO_PIN_5},
   {GPIOC, GPIO_PIN_4},
   {GPIOA, GPIO_PIN_7},
   {GPIOA, GPIO_PIN_6},
   {GPIOA, GPIO_PIN_5},
   {GPIOA, GPIO_PIN_4},
   {GPIOA, GPIO_PIN_3},
};

const pin_t matrix_in[] = {
   {GPIOB, GPIO_PIN_4},
   {GPIOB, GPIO_PIN_5},
   {GPIOB, GPIO_PIN_6},
   {GPIOB, GPIO_PIN_7},
};

volatile uint8_t matrix_button[(ARRAY_SIZE(matrix_in) * ARRAY_SIZE(matrix_row))/8];

void matrix_init(){
   //init row selector pins as output
   for(size_t i = 0; i < ARRAY_SIZE(matrix_row); i++){
      GPIO_InitTypeDef GPIO_InitStruct;
      GPIO_InitStruct.Pin = matrix_row[i].pin;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(matrix_row[i].port, &GPIO_InitStruct);
   }
   //init input pins with pulldown
   for(size_t i = 0; i < ARRAY_SIZE(matrix_in); i++){
      GPIO_InitTypeDef GPIO_InitStruct;
      GPIO_InitStruct.Pin = matrix_in[i].pin;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
      HAL_GPIO_Init(matrix_in[i].port, &GPIO_InitStruct);
   }
}

uint32_t matrix_next_row(){
   static uint32_t current_row = 0;
   //TODO: disable only last row
   //disable all rows
   for(size_t i = 0; i < ARRAY_SIZE(matrix_row); i++){
      HAL_GPIO_WritePin(matrix_row[i].port,matrix_row[i].pin,GPIO_PIN_RESET);
   }
   HAL_GPIO_WritePin(matrix_row[current_row].port,matrix_row[current_row].pin,GPIO_PIN_SET);
   uint32_t ret = current_row;
   current_row++;
   current_row %= ARRAY_SIZE(matrix_row);
   return ret;
}

void matrix_update(){
   uint32_t current_row = matrix_next_row();
   for(size_t i = 0; i < ARRAY_SIZE(matrix_in); i++){
      uint32_t bit = i + current_row * ARRAY_SIZE(matrix_in);
      HAL_GPIO_ReadPin(matrix_in[i].port,matrix_in[i].pin)?(matrix_button[(bit/8)]|=(1<<(bit%8))):(matrix_button[(bit/8)]&=~(1<<(bit%8)));
   }
}
