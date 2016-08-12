/** -*- C -*-
 * @file
 *
 * @brief Linear approximation to Steinhart-Hart function for thermistor
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

#include "steinhart.h"

static temp_point_t steinhart_approximation[] = { 
   {461, 253112},
   {699, 261213},
   {1012, 269532},
   {1355, 277286},
   {2344, 298127},
   {2626, 304780},
   {2865, 311134},
   {3105, 318600},
   {3307, 326264},
   {3477, 334360},
   {3618, 34303})
};

uint32_t steinhart(uint16_t adc_value)
{
   int size = sizeof(steinhart_approximation)/sizeof(temp_point_t);
   int index, upper = size, lower = 0;

   while(upper > lower) {
      index = lower + (upper - lower) / 2;
      if( adc_value >= steinhart_approximation[index].adc_value ) {
         if (adc_value < steinhart_approximation[index+1].adc_value)
            break;
         lower = index+1;
      } else if( adc_value < steinhart_approximation[index].adc_value ) {
         if (adc_value >= steinhart_approximation[index-1].adc_value) {
            index -= 1;
            break;
         }
         upper = index;
      }
   } 

   if (upper == lower || adc_value == steinhart_approximation[index].adc_value)
      return steinhart_approximation[index].temp_kelvin;
   
   uint32_t linear_approx =
      (steinhart_approximation[index+1].temp_kelvin -
       steinhart_approximation[index].temp_kelvin) *
      (adc_value - steinhart_approximation[index].adc_value) /
      (steinhart_approximation[index+1].adc_value -
       steinhart_approximation[index].adc_value) +
      steinhart_approximation[index].temp_kelvin;
   
   return linear_approx;
}

/* 
 * end of file
 */

