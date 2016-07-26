/** -*- C -*-
 * @file
 *
 * @brief Main file for thermo
 *
 * @page License
 *
 * Copyright (c) 2016, Dimitry Kloper.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include "stm32f0xx.h"
#include "stm32f0xx_misc.h"

#include "diag/Trace.h"

#include "hd44780.h"
#include "delay.h"
#include "adc.h"

static void
initialize(void)
{
   /* Enable clock for relevant peripherals */   
   RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN ;
   RCC->APB2ENR |= RCC_APB2ENR_ADCEN;   
   RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   
   NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
   NVIC_Init(&NVIC_InitStructure);

   NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
   NVIC_Init(&NVIC_InitStructure);   
}

static void clear_screen(void)
{
   hd44780_ir_write(HD44780_CMD_CLEAR_SCREEN);
   hd44780_wait_busy();
}

static void hd44780_putchar(char c)
{
   hd44780_dr_write(c);
   hd44780_wait_busy();
}

int
main(int argc, char* argv[])
{
   char buffer[80];
   
   initialize();
   
   hd44780_reset(HD44780_CMD_FUNC_SET |
                 HD44780_CMD_FUNC_2LINES);

   hd44780_ir_write(HD44780_CMD_DISPLAY         |
                HD44780_CMD_DISPLAY_ON      |
                HD44780_CMD_DISPLAY_CURS_ON |
                HD44780_CMD_DISPLAY_CURS_BLINK );
   hd44780_wait_busy();

   hd44780_ir_write(HD44780_CMD_EMS |
                HD44780_CMD_EMS_INCR);
   hd44780_wait_busy();

   adc_calibrate();
   adc_start();
   
   // Infinite loop
   while (1)
   {
      uint16_t internal_temp = adc_get(0),
               external_temp = adc_get(1),
                     battery = adc_get(2),
                   reference = adc_get(3);

      int len = snprintf(buffer, sizeof(buffer), "%04x %04x %04x %04x",
                         internal_temp, external_temp, battery, reference);
      
      clear_screen();
      for(int i = 0; i < len; i++)
        hd44780_putchar(buffer[i]);

      delay_1ms(500);
   }
}

/* 
 * end of file
 */

