/*
* This file is part of the stmbl project.
*
* Copyright (C) 2013 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2015 Ian McMahon <facetious.ian@gmail.com>
* Copyright (C) 2013 Nico Stute <crinq@crinq.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "sserial.h"
#include "crc8.h"
#include "defines.h"
#include "stm32f1xx_hal.h"
#include <string.h>

volatile uint8_t rxbuf[128];
volatile uint8_t txbuf[20];
uint16_t address;//current address pointer
int rxpos;
unit_no_t unit;
memory_t memory;
uint8_t *heap_ptr;
uint32_t timeout;
pd_table_t pd_table;
lbp_t lbp;
char name[] = LBPCardName;
int bufferpos;
int available;

uint8_t crc_reuest(uint8_t len) {
   uint8_t crc = crc8_init();
   for(int i = rxpos; i < rxpos+len; i++){
      crc = crc8_update(crc, (void*)&(rxbuf[i%sizeof(rxbuf)]), 1);  
   }
   crc8_finalize(crc);
   return crc == rxbuf[(rxpos+len)%sizeof(rxbuf)];
}

uint8_t crc8( uint8_t *addr, uint8_t len) {
   uint8_t crc = crc8_init();
   crc = crc8_update(crc, addr, len);
   return crc8_finalize(crc);
}

void send(uint8_t len, uint8_t docrc){
   timeout = 0;
   DMA1_Channel4->CCR &= (uint16_t)(~DMA_CCR_EN);
   if(docrc){
      txbuf[len] = crc8((uint8_t *)txbuf,len);
      DMA1_Channel4->CNDTR = len+1;
   }else{
      DMA1_Channel4->CNDTR = len;
   }
   DMA1_Channel4->CCR |= DMA_CCR_EN;
}

void init_hardware() {
   __HAL_RCC_USART1_CLK_ENABLE();
   UART_HandleTypeDef huart1;
   huart1.Instance = USART1;
   huart1.Init.BaudRate = 2500000;
   huart1.Init.WordLength = UART_WORDLENGTH_8B;
   huart1.Init.StopBits = UART_STOPBITS_1;
   huart1.Init.Parity = UART_PARITY_NONE;
   huart1.Init.Mode = UART_MODE_TX_RX;
   huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart1.Init.OverSampling = UART_OVERSAMPLING_16;
   HAL_UART_Init(&huart1);
   USART1->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
   
   GPIO_InitTypeDef GPIO_InitStruct;
   /**USART1 GPIO Configuration    
   PA9     ------> USART1_TX
   PA10     ------> USART1_RX 
   */
   GPIO_InitStruct.Pin = GPIO_PIN_9;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = GPIO_PIN_10;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
   
   __HAL_RCC_DMA1_CLK_ENABLE();
  
   //TX DMA
   DMA1_Channel4->CCR &= (uint16_t)(~DMA_CCR_EN);
   DMA1_Channel4->CPAR = (uint32_t)&(USART1->DR);
   DMA1_Channel4->CMAR = (uint32_t)txbuf;
   DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_DIR;
   DMA1->IFCR = DMA_IFCR_CTCIF4 | DMA_IFCR_CHTIF4 | DMA_IFCR_CGIF4;
   
   //RX DMA
   DMA1_Channel5->CCR &= (uint16_t)(~DMA_CCR_EN);
   DMA1_Channel5->CPAR = (uint32_t)&(USART1->DR);
   DMA1_Channel5->CMAR = (uint32_t)rxbuf;
   DMA1_Channel5->CNDTR = sizeof(rxbuf);
   DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_CIRC;
   DMA1->IFCR = DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CGIF5;
   DMA1_Channel5->CCR |= DMA_CCR_EN;
   
   //generate unit number from 96bit unique chip ID
   unit.unit = ((uint32_t *)UID_BASE)[0] ^ ((uint32_t *)UID_BASE)[1] ^ ((uint32_t *)UID_BASE)[2];
}

uint16_t add_pd(char *name_string, char *unit_string, uint8_t data_size_in_bits, uint8_t data_type, uint8_t data_dir, float param_min, float param_max) {
  process_data_descriptor_t pdr;
  pdr.record_type    = RECORD_TYPE_PROCESS_DATA_RECORD;
  pdr.data_size      = data_size_in_bits;
  pdr.data_type      = data_type;
  pdr.data_direction = data_dir;
  pdr.param_min      = param_min;
  pdr.param_max      = param_max;
  pdr.data_addr      = MEMPTR(*heap_ptr);
  
  heap_ptr += NUM_BYTES(data_size_in_bits);
  // this aligns the heap pointer to 32bit.  Not doing this causes the floats in the pd to be misaligned, which crashes the arm.
  if((uint32_t)heap_ptr % 4){
     heap_ptr += 4 - (uint32_t)heap_ptr % 4;
  }

  memcpy(heap_ptr, &pdr, sizeof(process_data_descriptor_t));
  // note that we don't store the names in the struct anymore.  The fixed-length struct is copied into memory, and then the nmaes go in directly behind it, so they'll read out properly

  uint16_t pd_ptr = MEMPTR(*heap_ptr); // save off the ptr to return, before we modify the heap ptr

  heap_ptr = (uint8_t *)&(((process_data_descriptor_t *)heap_ptr)->names);

  // copy the strings in after the pd
  strcpy((char *)heap_ptr, unit_string);
  heap_ptr += strlen(unit_string)+1;

  strcpy((char *)heap_ptr, name_string);
  heap_ptr += strlen(name_string)+1;

  // moved this up to before the pd record
  /*
  // this aligns the heap pointer to 32bit.  Not doing this causes the floats in the pd to be misaligned, which crashes the arm.
  if((uint32_t)heap_ptr % 4){
     heap_ptr += 4 - (uint32_t)heap_ptr % 4;
  }
  */

  return pd_ptr;
}

uint16_t add_mode(char *name_string, uint8_t index, uint8_t type) {
  mode_descriptor_t mdr;
  mdr.record_type = RECORD_TYPE_MODE_DATA_RECORD;
  mdr.index = index;
  mdr.type = type;//hw = 0, sw = 1
  mdr.unused = 0x00;

  memcpy(heap_ptr, &mdr, sizeof(mode_descriptor_t));

  uint16_t md_ptr = MEMPTR(*heap_ptr);

  heap_ptr = (uint8_t *)&(((mode_descriptor_t *)heap_ptr)->names);

  strcpy((char *)heap_ptr, name_string);
  heap_ptr += strlen(name_string)+1;

  return md_ptr;
}

void metadata(pd_metadata_t *pdm, process_data_descriptor_t *ptr) {
  pdm->ptr = ptr;
  pdm->range = ptr->data_type == DATA_TYPE_SIGNED ? MAX(ABS(ptr->param_min), ABS(ptr->param_max))*2 : ptr->param_max;
  pdm->bitmax = (1<<ptr->data_size)-1;
}

void sserial_init(){
  init_hardware();
  rxpos = 0;
  timeout = 1000;//make sure we start in timeout

  heap_ptr = memory.heap;

  uint16_t input_bits = 8; // this starts at 8 bits = 1 byte for the fault byte
  uint16_t output_bits = 0;

  // these are temp toc arrays that the macros will write pointers into.  the tocs get copied to main memory after everything else is written in
  uint16_t ptoc[32];
  uint16_t gtoc[32]; 

  uint16_t *ptocp = ptoc;
  uint16_t *gtocp = gtoc;
  process_data_descriptor_t *last_pd;

  ADD_PROCESS_VAR(("out", "none", 12, DATA_TYPE_BITS, DATA_DIRECTION_OUTPUT, 0, 1));       metadata(&(pd_table.output_pins), last_pd);
  //ADD_PROCESS_VAR(("enable", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_OUTPUT, 0, 1));             metadata(&(pd_table.enable), last_pd);
  //ADD_PROCESS_VAR(("pos_cmd", "rad", 16, DATA_TYPE_SIGNED, DATA_DIRECTION_OUTPUT, -3.2, 3.2));    metadata(&(pd_table.pos_cmd), last_pd);
  
  ADD_PROCESS_VAR(("in", "none", 20, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));    metadata(&(pd_table.input_pins), last_pd);
  
  ADD_PROCESS_VAR(("speed1", "none", 8, DATA_TYPE_UNSIGNED, DATA_DIRECTION_OUTPUT, 0, 100));    metadata(&(pd_table.pwm1), last_pd);
  ADD_PROCESS_VAR(("speed2", "none", 8, DATA_TYPE_UNSIGNED, DATA_DIRECTION_OUTPUT, 0, 100));    metadata(&(pd_table.pwm2), last_pd);
  
  //ADD_PROCESS_VAR(("fault", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));               metadata(&(pd_table.fault), last_pd);
  //ADD_PROCESS_VAR(("pos_fb", "rad", 16, DATA_TYPE_SIGNED, DATA_DIRECTION_INPUT, -3.2, 3.2));      metadata(&(pd_table.pos_fb), last_pd);
  //globals and modes are not working. https://github.com/LinuxCNC/linuxcnc/blob/2957cc5ad0a463c39fb35c10a0c14909c09a5fb7/src/hal/drivers/mesa-hostmot2/sserial.c#L1516
  // - globals need write support
  // - linuxcnc only supports globals of type DATA_TYPE_NONVOL_UNSIGNED or DATA_TYPE_NONVOL_SIGNED
  //ADD_GLOBAL_VAR(("swr", "non", 8, DATA_TYPE_NONVOL_UNSIGNED, DATA_DIRECTION_OUTPUT, 0, 0));

  //ADD_MODE(("foo", 0, 0));
  ADD_MODE(("IO Only", 0, 1));

  // automatically create padding pds based on the mod remainder of input/output bits
  if (input_bits % 8)  ADD_PROCESS_VAR(("padding", "", 8 -  (input_bits%8), DATA_TYPE_PAD, DATA_DIRECTION_INPUT,  0, 0));
  if (output_bits % 8) ADD_PROCESS_VAR(("padding", "", 8 - (output_bits%8), DATA_TYPE_PAD, DATA_DIRECTION_OUTPUT, 0, 0));

  // now that all the toc entries have been added, write out the tocs to memory and set up the toc pointers

  //calculate bytes from bits
  memory.discovery.input = input_bits >> 3;
  memory.discovery.output = output_bits >> 3;

  memory.discovery.ptocp = MEMPTR(*heap_ptr);

  for(uint8_t i = 0; i < ptocp - ptoc; i++) {
    *heap_ptr++ = ptoc[i] & 0x00FF; 
    *heap_ptr++ = (ptoc[i] & 0xFF00) >> 8;
  }
   // this is the ptoc end marker
  *heap_ptr++ = 0x00;
  *heap_ptr++ = 0x00;

  memory.discovery.gtocp = MEMPTR(*heap_ptr);

  for(uint8_t i = 0; i < gtocp - gtoc; i++) {
    *heap_ptr++ = gtoc[i] & 0x00FF; 
    *heap_ptr++ = (gtoc[i] & 0xFF00) >> 8;
  }
  // this is the gtoc end marker
  *heap_ptr++ = 0x00;
  *heap_ptr++ = 0x00;
}

void process_data_rpc(uint8_t fault, volatile uint8_t *input, volatile uint8_t *output) {
  uint16_t *ptocp = (uint16_t *)(memory.bytes + memory.discovery.ptocp);
  uint32_t local_rxpos = rxpos;
  *(input++) = fault;
  *input = 0x00;

  // data needs to be packed and unpacked based on its type and size
  // input is a pointer to the data that gets sent back to the host
  // need a bit pointer to keep track of partials

  uint8_t output_bit_ptr = 0;
  uint8_t input_bit_ptr = 0;

  while(*ptocp != 0x0000) {
    process_data_descriptor_t *pd = (process_data_descriptor_t *)(memory.bytes + *ptocp++);
    
    if (IS_INPUT(pd)) {
      uint16_t data_addr = pd->data_addr;
      uint8_t data_size = pd->data_size;
      uint8_t data_bit_ptr = 0;
      while(data_size > 0) {
        uint8_t bits_to_pack = data_size < BITSLEFT(input_bit_ptr) ? data_size : BITSLEFT(input_bit_ptr);
        if (BITSLEFT(data_bit_ptr) < bits_to_pack) { bits_to_pack = BITSLEFT(data_bit_ptr); }

        uint8_t mask = ((1<<bits_to_pack) - 1) << (data_bit_ptr);

        *input |= ((MEMU8(data_addr) & mask) >> data_bit_ptr) << input_bit_ptr;

        input_bit_ptr += bits_to_pack;
        data_bit_ptr += bits_to_pack;
        data_size -= bits_to_pack;
        if((input_bit_ptr %= 8) == 0) *(++input) = 0x00; // make sure we clear the input buffer whenever we increment bytes
        if((data_bit_ptr %= 8) == 0) data_addr++;
      }
    }
    if (IS_OUTPUT(pd)) {
      uint16_t data_addr = pd->data_addr;
      uint8_t data_size = pd->data_size;

      uint8_t val_bits_remaining = 8;
      uint8_t val = 0x00;

      while(data_size > 0) {
        // the number of bits to unpack this iteration is the number of bits remaining in the pd, or the number of bits remaining in the output byte, 
        // whichever is smaller.  Then, it can be even smaller if we have less room in the current val.

        uint8_t bits_to_unpack = data_size < BITSLEFT(output_bit_ptr) ? data_size : BITSLEFT(output_bit_ptr);
        if (val_bits_remaining < bits_to_unpack) { bits_to_unpack = val_bits_remaining; }

        // create a bitmask the width of the bits to read, shifted to the position in the output byte that we're pointing to
        uint8_t mask = ((1<<bits_to_unpack) - 1) << (output_bit_ptr);

        // val is what we get when we mask off output and then shift it to the proper place.  
        val = val | ((rxbuf[(local_rxpos+1)%sizeof(rxbuf)] & mask) >> (output_bit_ptr)) << (8-val_bits_remaining); 

        val_bits_remaining -= bits_to_unpack;
        data_size -= bits_to_unpack;
        output_bit_ptr += bits_to_unpack;
        // rxpos is a ringbuf and wraps around
        // note: this replaces the output argument
        if((output_bit_ptr %= 8) == 0){
           local_rxpos++;
           local_rxpos = local_rxpos % sizeof(rxbuf);
        }
        

        if(val_bits_remaining == 0 || data_size == 0) {
          MEMU8(data_addr++) = val;
          val_bits_remaining = 8;
          val = 0x00;
        }
      }
      // now we've finished unpacking it and storing it in memory, but we have to fix up the high bits if it wasn't a byte-aligned datasize.
      // for instance, if we receive 0xFFF in a 12 bit field, that is a negative number, but we stored it as 0x0FFF in memory.
      // strategy is to set the most significant n bits of the MSB to the most significant bit of the output value, iff the pd is defined as signed.
      if (SIGNED(pd) && pd->data_size % 8 != 0) {
        //printf("in output fixup.  data_addr %h  data_size %i num_bytes %i\n", pd->data_addr, pd->data_size, NUM_BYTES(pd->data_size));
        uint8_t msb_addr = pd->data_addr + NUM_BYTES(pd->data_size) - 1;
        //printf("in output fixup.  MSB (at %h): %h\n", msb_addr, MEMU8(msb_addr));
        
        // these two masks use data_size%8, this is the number of bits in the most significant byte, and since we tested for %8!=0 above, we know it's a partial byte
        if(MEMU8(msb_addr) & 1<<(pd->data_size%8 - 1)) { // this test uses a mask that is 1 in the most significant bit position, we only need to fixup the val if it's 1 (ie negative)
          // this mask is all the unused high bits set
          uint8_t mask = 0xFF ^ ((1<<pd->data_size%8) - 1);
          //printf("applying mask: %h\n", mask);
          MEMU8(msb_addr) |= mask;
        }

        //printf("fixed up val: %h\n", MEMU8(msb_addr));
      }
    }
  }
}


float scale_out(pd_metadata_t pd, int32_t val) {
  return val * pd.range / (float)pd.bitmax;
} 

int32_t scale_in(pd_metadata_t pd, float val) {
  return CLAMP(val, pd.ptr->param_min, pd.ptr->param_max) * pd.bitmax / pd.range;
}

//
// RT (
//   // update all hal pins with values from their associated pds
//
//
//   uint16_t foo = MEMU16(pd_table.pos_cmd.ptr->data_addr);
//   PIN(pos_cmd) = scale_out(pd_table.pos_cmd, *(int16_t*)&foo);
//   uint8_t outpins = MEMU8(pd_table.output_pins.ptr->data_addr);
//   PIN(out0) = outpins<<0 & 1 ? 1.0 : 0.0;
//   PIN(out1) = outpins<<1 & 1 ? 1.0 : 0.0;//TODO: out1 not working
//   PIN(out2) = outpins<<2 & 1 ? 1.0 : 0.0;
//   PIN(out3) = outpins<<3 & 1 ? 1.0 : 0.0;
//   uint8_t enable = MEMU8(pd_table.enable.ptr->data_addr);
//   PIN(enable) = enable<<0 & 1 ? 1.0 : 0.0;
//   //TODO: how to handle bidirectional pins properly?
//
//   *((uint16_t *)&(memory.bytes[pd_table.pos_fb.ptr->data_addr])) = (uint16_t)scale_in(pd_table.pos_fb, PIN(pos_fb));
//
//   MEMU8(pd_table.fault.ptr->data_addr) = BOOLPIN(fault);
//   MEMU8(pd_table.input_pins.ptr->data_addr) = BOOLPIN(in0)<<0 | BOOLPIN(in1)<<1 | BOOLPIN(in2)<<2 | BOOLPIN(in3)<<3;
//
// );
//

void sserial_do(){
   for(int j = 0;j<2;j++){
   //next received packet will be written to bufferpos
   bufferpos = sizeof(rxbuf) - DMA1_Channel5->CNDTR;
   //how many packets we have the the rx buffer for processing
   available = (bufferpos - rxpos + sizeof(rxbuf)) % sizeof(rxbuf);
   
   if (available >= 1) {
      lbp.byte = rxbuf[rxpos];

      if (lbp.ct == CT_LOCAL && lbp.wr == 0){ //local read, cmd+crc = 2b
         timeout = 0;
         if(available >= 2){
            switch(lbp.byte) {
               case LBPCookieCMD:
                 txbuf[0] = LBPCookie;
                 break;
               case LBPStatusCMD: //TODO: return status
                 txbuf[0] = 0x00;
                 break;
               case LBPCardName0Cmd ... LBPCardName3Cmd:
                 txbuf[0] = name[lbp.byte-LBPCardName0Cmd];
                 break;
               default: //TODO: handle unknown command condition
                 txbuf[0] = 0x00;
            }
            send(1,1);
            rxpos += 2;
         }else{
            continue;
         }
      }else if(lbp.ct == CT_LOCAL && lbp.wr == 1){//local write, cmd+data+crc = 3b
         timeout = 0;
         //0xFF and 0xFC are not followed by crc
         if(rxbuf[rxpos] == 0xFF){
            // reset parser
            rxpos += 1;
         }else if(rxbuf[rxpos] == 0xFC){
            // todo
            rxpos += 1;
         }else if(available >= 3){//writes do not expect crc in reply
            txbuf[0] = 0x00;
            send(1,0);
            rxpos += 3;  
         }else{
            continue;
         }
      }else if(lbp.ct == CT_RPC){//RPC TODO: check for ct should not required for rpc
          timeout = 0;
          if(lbp.byte == UnitNumberRPC && available >= 2){//unit number, cmd+crc = 2b
              txbuf[0] = unit.byte[0];
              txbuf[1] = unit.byte[1];
              txbuf[2] = unit.byte[2];
              txbuf[3] = unit.byte[3];
              send(4,1);
              rxpos += 2;
          }else if(lbp.byte == DiscoveryRPC && available >= 2){//discovery, cmd+crc = 2b
              memcpy((void*)txbuf,((uint8_t*)&memory.discovery),sizeof(memory.discovery));
              send(sizeof(memory.discovery),1);
              rxpos += 2;
          }else if(lbp.byte == ProcessDataRPC && available >= memory.discovery.output + 2){//process data, requires cmd+output bytes+crc
             //TODO: maybe packing and unpacking can be moved to RT
             process_data_rpc(0x00, txbuf, &(rxbuf[rxpos+1])); // todo: send a proper fault byte?
             uint32_t outpins = MEMU32(pd_table.output_pins.ptr->data_addr);
             send(memory.discovery.input,1);
             
             HAL_GPIO_WritePin(REL1_GPIO_Port,  REL1_Pin,  (outpins>>0 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
             HAL_GPIO_WritePin(REL2_GPIO_Port,  REL2_Pin,  (outpins>>1 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
             HAL_GPIO_WritePin(REL3_GPIO_Port,  REL3_Pin,  (outpins>>2 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
             HAL_GPIO_WritePin(REL4_GPIO_Port,  REL4_Pin,  (outpins>>3 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
             HAL_GPIO_WritePin(REL5_GPIO_Port,  REL5_Pin,  (outpins>>4 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
             HAL_GPIO_WritePin(REL6_GPIO_Port,  REL6_Pin,  (outpins>>5 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
             HAL_GPIO_WritePin(REL7_GPIO_Port,  REL7_Pin,  (outpins>>6 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
             HAL_GPIO_WritePin(REL8_GPIO_Port,  REL8_Pin,  (outpins>>7 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
             HAL_GPIO_WritePin(REL9_GPIO_Port,  REL9_Pin,  (outpins>>8 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
             HAL_GPIO_WritePin(REL10_GPIO_Port, REL10_Pin, (outpins>>9 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
             HAL_GPIO_WritePin(REL11_GPIO_Port, REL11_Pin, (outpins>>10 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
             HAL_GPIO_WritePin(REL12_GPIO_Port, REL12_Pin, (outpins>>11 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
             
             uint8_t pwm1 = MEMU8(pd_table.pwm1.ptr->data_addr);
             uint8_t pwm2 = MEMU8(pd_table.pwm2.ptr->data_addr);
             TIM3->CCR2 = pwm1 * 14;
             TIM3->CCR3 = pwm2 * 14;
             
             MEMU8(pd_table.input_pins.ptr->data_addr) =
                HAL_GPIO_ReadPin(IN1_GPIO_Port,  IN1_Pin)<<0 |
                HAL_GPIO_ReadPin(IN2_GPIO_Port,  IN2_Pin)<<1 |
                HAL_GPIO_ReadPin(IN3_GPIO_Port,  IN3_Pin)<<2 |
                HAL_GPIO_ReadPin(IN4_GPIO_Port,  IN4_Pin)<<3 |
                HAL_GPIO_ReadPin(IN5_GPIO_Port,  IN5_Pin)<<4 |
                HAL_GPIO_ReadPin(IN6_GPIO_Port,  IN6_Pin)<<5 |
                HAL_GPIO_ReadPin(IN7_GPIO_Port,  IN7_Pin)<<6 |
                HAL_GPIO_ReadPin(IN8_GPIO_Port,  IN8_Pin)<<7;
             
             MEMU8(pd_table.input_pins.ptr->data_addr+1) =
                HAL_GPIO_ReadPin(IN9_GPIO_Port,  IN9_Pin )<<0 |
                HAL_GPIO_ReadPin(IN10_GPIO_Port, IN10_Pin)<<1 |
                HAL_GPIO_ReadPin(IN11_GPIO_Port, IN11_Pin)<<2 |
                HAL_GPIO_ReadPin(IN12_GPIO_Port, IN12_Pin)<<3 |
                HAL_GPIO_ReadPin(IN13_GPIO_Port, IN13_Pin)<<4 |
                HAL_GPIO_ReadPin(IN14_GPIO_Port, IN14_Pin)<<5 |
                HAL_GPIO_ReadPin(IN15_GPIO_Port, IN15_Pin)<<6 |
                HAL_GPIO_ReadPin(IN16_GPIO_Port, IN16_Pin)<<7;
             
             MEMU8(pd_table.input_pins.ptr->data_addr+2) =
                HAL_GPIO_ReadPin(IN17_GPIO_Port,  IN17_Pin)<<0 |
                HAL_GPIO_ReadPin(IN18_GPIO_Port,  IN18_Pin)<<1 |
                HAL_GPIO_ReadPin(IN19_GPIO_Port,  IN19_Pin)<<2 |
                HAL_GPIO_ReadPin(IN20_GPIO_Port,  IN20_Pin)<<3;
             
             //uint16_t foo = MEMU16(pd_table.pos_cmd.ptr->data_addr);
             //float p = scale_out(pd_table.pos_cmd, *(int16_t*)&foo);
             //PIN(pos_cmd_d) = minus(p,last_pos_cmd)*1000.0f;//TODO: only valid for 1khz servo thread
             //TODO:########################################################################################################################
             //we cannot send the reply based on crc, as this causes timeouts
             //instead we should check for errors in RT
             //if(!crc_reuest(memory.discovery.output + 1)){
            //    PIN(crc_error)++;
             //}
             rxpos += memory.discovery.output + 2;
          }else{
             continue;
          }
      }else if (lbp.ct == CT_RW && lbp.wr == 0){ //read TODO: implement write.
         timeout = 0;
         if(available >= 2){
            if (lbp.as == 1){ //address included in command = cmd+addr+addr+crc
               if (available >= 4) {
                  address = rxbuf[(rxpos+1)%sizeof(rxbuf)] + (rxbuf[(rxpos+2)%sizeof(rxbuf)]<<8);
                  rxpos += 4;
               }else{
                  continue;
               }
            }else{ //address not included in command = cmd+crc
               rxpos += 2;
            }
            //TODO: check if address is valid
            memcpy((void*)txbuf,&memory.bytes[address],(1<<lbp.ds));
            send((1<<lbp.ds),1);
            if(lbp.ai == 1){//auto increment address by datasize
               address += (1<<lbp.ds);
            }
         }else{
            continue;
         }
      } else {
         //TODO: handle unkown packet
         //TODO###############################################################################################################
         //PIN(error)++;
      }
   }

   timeout++;
   if(timeout > 100.0){//TODO: clamping
      HAL_GPIO_WritePin(YELLOW_GPIO_Port,YELLOW_Pin,GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(REL1_GPIO_Port,  REL1_Pin,  GPIO_PIN_RESET);
      HAL_GPIO_WritePin(REL2_GPIO_Port,  REL2_Pin,  GPIO_PIN_RESET);
      HAL_GPIO_WritePin(REL3_GPIO_Port,  REL3_Pin,  GPIO_PIN_RESET);
      HAL_GPIO_WritePin(REL4_GPIO_Port,  REL4_Pin,  GPIO_PIN_RESET);
      HAL_GPIO_WritePin(REL5_GPIO_Port,  REL5_Pin,  GPIO_PIN_RESET);
      HAL_GPIO_WritePin(REL6_GPIO_Port,  REL6_Pin,  GPIO_PIN_RESET);
      HAL_GPIO_WritePin(REL7_GPIO_Port,  REL7_Pin,  GPIO_PIN_RESET);
      HAL_GPIO_WritePin(REL8_GPIO_Port,  REL8_Pin,  GPIO_PIN_RESET);
      HAL_GPIO_WritePin(REL9_GPIO_Port,  REL9_Pin,  GPIO_PIN_RESET);
      HAL_GPIO_WritePin(REL10_GPIO_Port, REL10_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(REL11_GPIO_Port, REL11_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(REL12_GPIO_Port, REL12_Pin, GPIO_PIN_RESET);
      TIM3->CCR2 = 0;
      TIM3->CCR3 = 0;
      //PIN(connected) = 0;
      rxpos = bufferpos;
   }else{
      HAL_GPIO_WritePin(YELLOW_GPIO_Port,YELLOW_Pin,GPIO_PIN_RESET);
      //PIN(connected) = 1;
   }
   rxpos = rxpos % sizeof(rxbuf);
}
}
