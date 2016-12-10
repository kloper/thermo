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
#include "steinhart.h"
#include "thermocouple.h"

static void
initialize(void)
{
   RCC_DeInit();
   
   /* Enable clock for relevant peripherals */   
   RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOFEN;
   RCC->APB2ENR |= RCC_APB2ENR_ADCEN;   
   RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   
   NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
   NVIC_Init(&NVIC_InitStructure);

   NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
   NVIC_Init(&NVIC_InitStructure);
}

static void hd44780_clear_screen(void)
{
   hd44780_ir_write(HD44780_CMD_CLEAR_SCREEN);
   hd44780_wait_busy();
}

static void hd44780_putchar(char c)
{
   hd44780_dr_write((uint8_t)c);
   hd44780_wait_busy();
}

static void hd44780_puts(const char *str, int size)
{
   __disable_irq();
   for(int i = 0; i < size; i++)
      hd44780_putchar(str[i]);
   __enable_irq();
}

static void hd44780_goto_addr(uint8_t addr)
{
   __disable_irq();
   hd44780_ir_write(HD44780_CMD_SET_DDRAM_ADDR | addr);
   hd44780_wait_busy();
   __enable_irq();
}


int
main(int argc, char* argv[])
{
   char buffer[80];
   
   initialize();
   
   hd44780_reset(HD44780_CMD_FUNC_SET |
                 HD44780_CMD_FUNC_2LINES);

   hd44780_ir_write(HD44780_CMD_DISPLAY |
                HD44780_CMD_DISPLAY_ON |
                HD44780_CMD_DISPLAY_CURS_ON |
                HD44780_CMD_DISPLAY_CURS_BLINK );
   hd44780_wait_busy();

   hd44780_ir_write(HD44780_CMD_EMS |
                HD44780_CMD_EMS_INCR);
   hd44780_wait_busy();

   adc_calibrate();
   adc_start();
   
   while(1)
   {
      uint16_t internal_temp = adc_get_median(0),
               external_temp = adc_get_median(1),
                     battery = adc_get_median(2),
                   reference = adc_get_median(3);
      
      //trace_printf("%d %d %d\n", external_temp, internal_temp, reference);
      uint32_t vdd = 3300u * *VREFINT_CAL / reference;
      uint32_t internal_temp_volts = internal_temp * vdd / 0xfff,
               external_temp_volts = external_temp * vdd / 0xfff,
                     battery_volts = battery * vdd / 0xfff;
        
      int len = snprintf(buffer, sizeof(buffer), "%04x %04x %04x %04x",
                         internal_temp, external_temp, battery, reference);
      
      //hd44780_clear_screen();
      hd44780_goto_addr(0);
      hd44780_puts(buffer, len);

      len = snprintf(buffer, sizeof(buffer), "%ld.%03ld %ld.%03ld %ld.%03ld",
                     internal_temp_volts / 1000,
                     internal_temp_volts % 1000,
                     external_temp_volts / 1000,
                     external_temp_volts % 1000,
                     battery_volts / 1000,
                     battery_volts % 1000);
      hd44780_goto_addr(64);
      hd44780_puts(buffer, len);

      int32_t internal_temp_celsius =
         (int32_t)steinhart(internal_temp) - 273150;
      int32_t external_temp_celsius =
         hot_junction_temperature(internal_temp, external_temp, reference);
      
      len = snprintf(buffer, sizeof(buffer), "%ld.%03d %ld.%05d",
                     internal_temp_celsius / 1000,
                     abs(internal_temp_celsius) % 1000,
                     external_temp_celsius / 100000,
                     abs(external_temp_celsius) % 100000);
      hd44780_goto_addr(20);
      hd44780_puts(buffer, len);

      len = snprintf(buffer, sizeof(buffer), "%lu.%03lu",
                     vdd / 1000,
                     vdd % 1000);
      hd44780_goto_addr(84);
      hd44780_puts(buffer, len);
      
      delay_1ms(1000);
   }
}

/* 
 * end of file
 */

