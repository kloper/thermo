/** -*- C -*-
 * @file
 *
 * @brief Evaluate value of function based on its linear approximation
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

#include "linapprox.h"

int32_t linapprox(const point_t *const values,
                  const int size,
                  int32_t x)
{
   int index, upper = size-1, lower = 0;

   while(upper > lower) {
      index = lower + (upper - lower) / 2;
      if( x >= values[index].x ) {
         if (x < values[index+1].x)
            break;
         lower = index+1;
      } else if( x < values[index].x ) {
         if (x >= values[index-1].x) {
            index -= 1;
            break;
         }
         upper = index;
      }
   } 

   if (upper == lower || x == values[index].x)
      return values[index].y;
   
   int32_t linear_approx =
      (values[index+1].y - values[index].y) * (x - values[index].x) /
      (values[index+1].x - values[index].x) +
      values[index].y;
   
   return linear_approx;
}


/* 
 * end of file
 */

