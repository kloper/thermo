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

static void hd44780_nibble_out(uint8_t nibble);
static uint8_t hd44780_nibble_in(void);
static void hd44780_byte_out(uint8_t value);
static uint8_t hd44780_byte_in(void);

void hd44780_reset(uint8_t cmd)
{
   GPIOA->MODER = (GPIOA->MODER & ~HD44780_CLRMODE_MASK) | HD44780_OUTMODE_MASK;
   GPIOA->OTYPER &= ~HD44780_OUT_MASK;
   GPIOA->OSPEEDR &= ~HD44780_CLRMODE_MASK; /* low speed */
   GPIOA->PUPDR &= ~HD44780_CLRMODE_MASK; /* no push/pull ups */
   GPIOA->BRR = HD44780_OUT_MASK;

   GPIOB->MODER = 1<<(2*HD44780_RST);
   GPIOB->OTYPER &= ~(1<<HD44780_RST);
   GPIOB->OSPEEDR &= ~(1<<HD44780_RST);
   GPIOB->PUPDR &= ~(1<<HD44780_RST);
   GPIOB->BRR = 1<<HD44780_RST;

   if( cmd == 0xff )
      return;

   /* wait for display to shut down */
   for(uint32_t i=1500000; i > 0; i--)
      __NOP();

   /* turn RST up and wait for DB7 to go up */
   GPIOB->BSRR = 1<<HD44780_RST;
   hd44780_wait_busy();

   hd44780_ir_write(cmd);
   hd44780_wait_busy();

   hd44780_nibble_out(3);
   for(uint32_t i=20000; i > 0; i--)
      __NOP();
   
   hd44780_ir_write(cmd);
   hd44780_wait_busy();

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
   GPIOA->BSRR = 1<<HD44780_EN;
   GPIOA->BSRR = nibble_to_bsrr[nibble & 0xf];
   __NOP();
   __NOP();
   __NOP();
   GPIOA->BRR = 1<<HD44780_EN;
}

static uint8_t hd44780_nibble_in()
{
   uint16_t data_in = 0;
   
   GPIOA->BSRR = 1<<HD44780_EN;
   __NOP();
   __NOP();
   __NOP();
   data_in = GPIOA->IDR;
   GPIOA->BRR = 1<<HD44780_EN;

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
   GPIOA->MODER = (GPIOA->MODER & ~HD44780_CLRMODE_MASK) | HD44780_INMODE_MASK;

   uint8_t nibble0 = hd44780_nibble_in();
   uint8_t nibble1 = hd44780_nibble_in();

   GPIOA->MODER = (GPIOA->MODER & ~HD44780_CLRMODE_MASK) | HD44780_OUTMODE_MASK;

   return (uint8_t)((nibble0 << 4) | (nibble1 & 0xf));
}

uint8_t hd44780_wait_busy(void)
{
   GPIOA->BSRR = (0x10000 << HD44780_RS) | (1 << HD44780_RW);
   uint8_t value = 0;
   do {
       value = hd44780_byte_in();
   } while(value & 0x80);

   return value;
}

void hd44780_ir_write(uint8_t ir)
{
   GPIOA->BSRR = (0x10000 << HD44780_RS) | (0x10000 << HD44780_RW);
   hd44780_byte_out(ir);
}

void hd44780_dr_write(uint8_t dr)
{
   GPIOA->BSRR = (1 << HD44780_RS) | (0x10000 << HD44780_RW);
   hd44780_byte_out(dr);
}

uint8_t hd44780_dr_read()
{
   GPIOA->BSRR = (1 << HD44780_RS) | (1 << HD44780_RW);
   return hd44780_byte_in();
}

/* 
 * end of file
 */

