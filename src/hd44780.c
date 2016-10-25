/** -*- C -*-
 * @file
 *
 * @brief Hitachi HD44780 LED display driver
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

#include "hd44780.h"
#include "delay.h"

static void hd44780_nibble_out(uint8_t nibble);
static uint8_t hd44780_nibble_in(void);
static void hd44780_byte_out(uint8_t value);
static uint8_t hd44780_byte_in(void);

#define nop_delay(n) \
   do { int __tmp = n; do { __NOP(); } while(__tmp--); } while(0)

void hd44780_reset(uint8_t cmd)
{
   GPIOA->MODER &= ~((3U<<(2*HD44780_DB4)) |
                     (3U<<(2*HD44780_DB5)) |
                     (3U<<(2*HD44780_DB6)) |
                     (3U<<(2*HD44780_DB7)));
   
   GPIOA->MODER |= (1<<(2*HD44780_DB4)) |
                   (1<<(2*HD44780_DB5)) |
                   (1<<(2*HD44780_DB6)) |
                   (1<<(2*HD44780_DB7));
   
   GPIOA->OTYPER &= ~((1<<HD44780_DB4) |
                      (1<<HD44780_DB5) |
                      (1<<HD44780_DB6) |
                      (1<<HD44780_DB7));
   
   GPIOA->OSPEEDR &= ~((3U<<(2*HD44780_DB4)) |
                       (3U<<(2*HD44780_DB5)) |
                       (3U<<(2*HD44780_DB6)) |
                       (3U<<(2*HD44780_DB7)));  /* low speed */
   
   GPIOA->PUPDR &= ~((3U<<(2*HD44780_DB4)) |
                     (3U<<(2*HD44780_DB5)) |
                     (3U<<(2*HD44780_DB6)) |
                     (3U<<(2*HD44780_DB7))); /* no push/pull ups */
   
   GPIOA->BRR = (1<<HD44780_DB4) |
                (1<<HD44780_DB5) |
                (1<<HD44780_DB6) |
                (1<<HD44780_DB7); /* clear all bits */

   GPIOB->MODER &= ~(3U<<(2*HD44780_EN));
   GPIOB->MODER |= 1<<(2*HD44780_EN);
   GPIOB->OTYPER &= ~(1<<HD44780_EN);
   GPIOB->OSPEEDR &= ~(3U<<(2*HD44780_EN));
   GPIOB->PUPDR &= ~(3U<<(2*HD44780_EN));
   GPIOB->BRR = 1<<HD44780_EN;

   GPIOF->MODER &= ~((3U<<(2*HD44780_RS)) |
                     (3U<<(2*HD44780_RW)));
   
   GPIOF->MODER |= (1<<(2*HD44780_RS)) |
                   (1<<(2*HD44780_RW));
   
   GPIOF->OTYPER &= ~((1<<HD44780_RS) |
                      (1<<HD44780_RW));
   
   GPIOF->OSPEEDR &= ~((3U<<(2*HD44780_RS)) |
                       (3U<<(2*HD44780_RW)));  /* low speed */
   
   GPIOF->PUPDR &= ~((3U<<(2*HD44780_RS)) |
                     (3U<<(2*HD44780_RW))); /* no push/pull ups */
   
   GPIOF->BRR = (1<<HD44780_RS) |
                (1<<HD44780_RW); /* clear all bits */
   
   if( cmd == 0xff )
      return;

   /* wait for display to complete boot */
   delay_1ms(100);

   /* bPower-on initialization, switching from 8-bit to 4-bit mode.
      See [HD44780 Data-sheet](http://tinyurl.com/qxfogu9) see
      "Initializing by Instruction" section Table 12 */
   
   hd44780_nibble_out(3);
   delay_1ms(100);

   hd44780_nibble_out(3);
   delay_1ms(100);

   hd44780_nibble_out(3);
   delay_1ms(100);

   hd44780_nibble_out(cmd >> 4);
   delay_1ms(100);
      
   hd44780_ir_write(cmd);
   hd44780_wait_busy();
}

static uint32_t nibble_to_bsrr[16] = {
   /* 0  */ (0x10000<<HD44780_DB4)|(0x10000<<HD44780_DB5)|
            (0x10000<<HD44780_DB6)|(0x10000<<HD44780_DB7),
   /* 1  */ (0x00001<<HD44780_DB4)|(0x10000<<HD44780_DB5)|
            (0x10000<<HD44780_DB6)|(0x10000<<HD44780_DB7),
   /* 2  */ (0x10000<<HD44780_DB4)|(0x00001<<HD44780_DB5)|
            (0x10000<<HD44780_DB6)|(0x10000<<HD44780_DB7),
   /* 3  */ (0x00001<<HD44780_DB4)|(0x00001<<HD44780_DB5)|
            (0x10000<<HD44780_DB6)|(0x10000<<HD44780_DB7),
   /* 4  */ (0x10000<<HD44780_DB4)|(0x10000<<HD44780_DB5)|
            (0x00001<<HD44780_DB6)|(0x10000<<HD44780_DB7),
   /* 5  */ (0x00001<<HD44780_DB4)|(0x10000<<HD44780_DB5)|
            (0x00001<<HD44780_DB6)|(0x10000<<HD44780_DB7),
   /* 6  */ (0x10000<<HD44780_DB4)|(0x00001<<HD44780_DB5)|
            (0x00001<<HD44780_DB6)|(0x10000<<HD44780_DB7),
   /* 7  */ (0x00001<<HD44780_DB4)|(0x00001<<HD44780_DB5)|
            (0x00001<<HD44780_DB6)|(0x10000<<HD44780_DB7),
   /* 8  */ (0x10000<<HD44780_DB4)|(0x10000<<HD44780_DB5)|
            (0x10000<<HD44780_DB6)|(0x00001<<HD44780_DB7),
   /* 9  */ (0x00001<<HD44780_DB4)|(0x10000<<HD44780_DB5)|
            (0x10000<<HD44780_DB6)|(0x00001<<HD44780_DB7),
   /* 10 */ (0x10000<<HD44780_DB4)|(0x00001<<HD44780_DB5)|
            (0x10000<<HD44780_DB6)|(0x00001<<HD44780_DB7),
   /* 11 */ (0x00001<<HD44780_DB4)|(0x00001<<HD44780_DB5)|
            (0x10000<<HD44780_DB6)|(0x00001<<HD44780_DB7),
   /* 12 */ (0x10000<<HD44780_DB4)|(0x10000<<HD44780_DB5)|
            (0x00001<<HD44780_DB6)|(0x00001<<HD44780_DB7),
   /* 13 */ (0x00001<<HD44780_DB4)|(0x10000<<HD44780_DB5)|
            (0x00001<<HD44780_DB6)|(0x00001<<HD44780_DB7),
   /* 14 */ (0x10000<<HD44780_DB4)|(0x00001<<HD44780_DB5)|
            (0x00001<<HD44780_DB6)|(0x00001<<HD44780_DB7),
   /* 15 */ (0x00001<<HD44780_DB4)|(0x00001<<HD44780_DB5)|
            (0x00001<<HD44780_DB6)|(0x00001<<HD44780_DB7),
};

static void hd44780_nibble_out(uint8_t nibble)
{
   GPIOB->BSRR = 1<<HD44780_EN;
   nop_delay(100);
   GPIOA->BSRR = nibble_to_bsrr[nibble & 0xf];
   nop_delay(100);
   GPIOB->BRR = 1<<HD44780_EN;
   nop_delay(100);
}

static uint8_t hd44780_nibble_in()
{
   uint16_t data_in = 0;

   GPIOB->BSRR = 1<<HD44780_EN;
   nop_delay(100);
   data_in = GPIOA->IDR;
   GPIOB->BRR = 1<<HD44780_EN;
   
   return
      (((data_in>>HD44780_DB4) & 1) << 0) |
      (((data_in>>HD44780_DB5) & 1) << 1) |
      (((data_in>>HD44780_DB6) & 1) << 2) |
      (((data_in>>HD44780_DB7) & 1) << 3);
}

static void hd44780_byte_out(uint8_t value)
{
   hd44780_nibble_out(value>>4);
   hd44780_nibble_out(value);
}

static uint8_t hd44780_byte_in(void)
{
   GPIOA->MODER &= ~((3U<<(2*HD44780_DB4)) |
                     (3U<<(2*HD44780_DB5)) |
                     (3U<<(2*HD44780_DB6)) |
                     (3U<<(2*HD44780_DB7)));
   
   
   uint8_t nibble0 = hd44780_nibble_in();
   uint8_t nibble1 = hd44780_nibble_in();

   GPIOA->MODER |= (1<<(2*HD44780_DB4)) |
                   (1<<(2*HD44780_DB5)) |
                   (1<<(2*HD44780_DB6)) |
                   (1<<(2*HD44780_DB7));
   
   return (uint8_t)((nibble0 << 4) | (nibble1 & 0xf));
}

uint8_t hd44780_wait_busy(void)
{
   GPIOF->BSRR = (0x10000 << HD44780_RS) | (1 << HD44780_RW);

   uint8_t value = 0;
   do {
       value = hd44780_byte_in();
   } while(value & 0x80);

   GPIOF->BSRR = (0x10000 << HD44780_RS) | (0x10000 << HD44780_RW);

   return value;
}

void hd44780_ir_write(uint8_t ir)
{
   GPIOF->BSRR = (0x10000 << HD44780_RS) | (0x10000 << HD44780_RW);
   hd44780_byte_out(ir);
}

void hd44780_dr_write(uint8_t dr)
{
   GPIOF->BSRR = (1 << HD44780_RS) | (0x10000 << HD44780_RW);
   hd44780_byte_out(dr);
}

uint8_t hd44780_dr_read()
{
   GPIOF->BSRR = (1 << HD44780_RS) | (1 << HD44780_RW);
   return hd44780_byte_in();
}

/* 
 * end of file
 */

