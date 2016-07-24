/** -*- C -*-
 * @file
 *
 * @brief Primitive delay implementation based on TIM3
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

#include "stm32f0xx.h"
#include "delay.h"

static uint32_t tim3_tick = 0;

void TIM3_IRQHandler(void)
{
  tim3_tick = 1;
  TIM3->SR &= ~TIM_SR_UIF;
}

void delay_start(uint16_t count, uint16_t prescale)
{
   TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_OPM | TIM_CR1_URS;
   TIM3->DIER = TIM_DIER_UIE;
   TIM3->CNT = 1;
   TIM3->PSC = prescale;
   TIM3->ARR = count;

   tim3_tick = 0;
   TIM3->SR &= ~TIM_SR_UIF;
   TIM3->CR1 |= TIM_CR1_CEN;
}

void delay_wait(void)
{
   while(!tim3_tick) {
      __WFI();
   }
}

void delay(uint16_t count, uint16_t prescale)
{
   delay_start(count, prescale);
   delay_wait();
}

/* 
 * end of file
 */

