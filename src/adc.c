/** -*- C -*-
 * @file
 *
 * @brief ADC sampling of voltages from temperature and battery sensors
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

#include <string.h>

#include "stm32f0xx.h"

#include "adc.h"
#include "delay.h"

typedef struct _adc_median_array {
   uint16_t head;
   uint16_t reserved;
   uint16_t data[ADC_MEDIAN_WINDOW];
} adc_median_array_t;

static uint16_t adc_values[ADC_CHANNELS_NUM] = {0};
static uint8_t adc_values_index = 0;

static uint16_t adc_average_size = 100;
static uint32_t adc_average[ADC_CHANNELS_NUM] = {0};
static uint16_t adc_avg_count[ADC_CHANNELS_NUM] = {0};
static adc_median_array_t adc_median[ADC_CHANNELS_NUM] = {0};

static void adc_median_put(const uint8_t index, const uint16_t value)
{
   adc_median_array_t *median = adc_median + index;

   median->data[median->head] = value;
   median->head = (median->head + 1U) % ADC_MEDIAN_WINDOW;
}

void adc_stop(void)
{
   if( ADC1->CR & ADC_CR_ADSTART ) {
      ADC1->CR |= ADC_CR_ADSTP;
      while( ADC1->CR & ADC_CR_ADSTP )
         delay_1ms(1);      
   }
   ADC1->CR |= ADC_CR_ADDIS;
   while( ADC1->CR & ADC_CR_ADEN )
      delay_1ms(1);      
}

void adc_calibrate(void)
{
   if( ADC1->CR & ADC_CR_ADEN )
      adc_stop();
   
   ADC1->CR |= ADC_CR_ADCAL;
   while( ADC1->CR & ADC_CR_ADCAL)
      delay_1ms(1);
}

void
adc_start(void)
{
   /* Enable conversion end and sequence end interrupts */
   ADC1->IER = ADC_IER_EOSEQIE | ADC_IER_EOCIE;

   /* continuous ADC scan */
   ADC1->CFGR1 = ADC_CFGR1_CONT;

   /* run on slow clock - PCLK/4 */
   ADC1->CFGR2 = ADC_CFGR2_CKMODE_1;

   /* 0b100 - 239.5 ADC clock cycles */
   ADC1->SMPR = ADC_SMPR_SMP_2 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_0;

   /* ADC channels 0, 1, 2 and 17 (Vref) */
   ADC1->CHSELR = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 |
                 ADC_CHSELR_CHSEL2 | ADC_CHSELR_CHSEL17;

   /* Enable internal voltage reference */
   ADC->CCR = ADC_CCR_VREFEN;

   /* enable ADC and wait till it is ready */
   ADC1->CR |= ADC_CR_ADEN;
   while( !(ADC1->ISR & ADC_ISR_ADRDY) )
      delay_1ms(1);

   ADC1->CR |= ADC_CR_ADSTART;
}

uint16_t adc_get(uint8_t index)
{
   uint16_t res = 0;
   
   __disable_irq();

   res = adc_values[index];
   
   __enable_irq();

   return res;
}

uint16_t adc_get_avg(uint8_t index)
{
   uint32_t res = 0;
   
   __disable_irq();

   res = adc_average[index] / adc_avg_count[index];
   
   __enable_irq();

   return (uint16_t)res;
}

uint16_t adc_get_median(uint8_t index)
{
   adc_median_array_t *median = adc_median + index;
   uint16_t data[ADC_MEDIAN_WINDOW] = {0};

   __disable_irq();

   memcpy(data, median->data, sizeof(median->data));
   
   __enable_irq();

   for( unsigned int i = 0; i < ADC_MEDIAN_WINDOW; i++ ) {
      for( unsigned int j = 0; j < (ADC_MEDIAN_WINDOW - 1 - i); j++ ) {
         if( data[j] > data[j+1] ) {
            uint16_t tmp = data[j+1];
            data[j+1] = data[j];
            data[j] = tmp;
         }
      }
   }

   return data[ADC_MEDIAN_WINDOW / 2 + 1];
}

void ADC1_IRQHandler(void)
{
   if( ADC1->ISR & ADC_ISR_EOC ) {
      if( !(ADC1->ISR & ADC_ISR_OVR) ) {
         adc_values[adc_values_index] = (uint16_t)ADC1->DR;
         if(adc_avg_count[adc_values_index] >= adc_average_size) {
            adc_median_put(adc_values_index,
                           (uint16_t)(adc_average[adc_values_index] /
                                      adc_avg_count[adc_values_index]));
            
            adc_average[adc_values_index] = 0;
            adc_avg_count[adc_values_index] = 0;
         } 
         adc_avg_count[adc_values_index]++;
         adc_average[adc_values_index] += adc_values[adc_values_index];
         
         adc_values_index++;
      }
   }
   
   if( ADC1->ISR & ADC_ISR_EOS ) {
      adc_values_index = 0;
      ADC1->ISR |= ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
   }
}

/* 
 * end of file
 */

