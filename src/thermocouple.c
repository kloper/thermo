/** -*- C -*-
 * @file
 *
 * @brief Convert ADC voltage to temperature based on Thermocouple properties
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

#include "adc.h"
#include "linapprox.h"
#include "thermocouple.h"

static point_t t2cjv[] = {
   {400, -69496},
   {426, -65036},
   {454, -60442},
   {512, -51518},
   {575, -42564},
   {643, -33589},
   {716, -24589},
   {795, -15442},
   {880, -6151},
   {971, 3291},
   {1049, 11052},
   {1132, 19038},
   {1221, 27351},
   {1319, 36271},
   {1504, 52649},
   {1977, 93779},
   {2108, 105441},
   {2225, 116112},
   {2347, 127585},
   {2459, 138516},
   {2564, 149196},
   {2663, 159733},
   {2758, 170358},
   {2848, 180979},
   {2932, 191472},
   {3012, 202082},
   {3088, 212821},
   {3159, 223542},
   {3227, 234548},
   {3259, 240014},
   {3291, 245687},
   {3331, 253097},
   {3369, 260501},
   {3406, 268093},
   {3442, 275887},
   {3476, 283661},
   {3509, 291641},
   {3541, 299839},
   {3571, 307989},
   {3600, 316350},
   {3628, 324933},
   {3655, 333749},
   {3680, 342449},
   {3705, 351736},
   {3728, 360873},
   {3751, 370660},
   {3772, 380247},
   {3792, 390041},
   {3802, 395210},
   {3812, 400580},
   {3830, 410802},
   {3839, 416208},
   {3848, 421831},
   {3857, 427690},
   {3865, 433110},
   {3873, 438748},
   {3881, 444619},
   {3889, 450744},
   {3896, 456328},
   {3903, 462140},
   {3910, 468197},
   {3917, 474521},
   {3923, 480171},
   {3930, 487054},
   {3936, 493224},
   {3942, 499667},
   {3948, 506405},
   {3954, 513467},
   {3960, 520883},
   {3966, 528688},
   {3971, 535519},
   {3976, 542676},
   {3981, 550189},
   {3986, 558093},
   {3990, 564726},
   {3995, 573443},
   {3999, 580789}
};

static int32_t cold_junction_voltage(uint16_t adc_value)
{
   const int size = sizeof(t2cjv) / sizeof(point_t);
   return linapprox(t2cjv, size, (int32_t)adc_value);
}

static point_t hjv2t[] = {
   {-600000, -20745761},
   {-583899, -19663385},
   {-565399, -18565988},
   {-544499, -17453017},
   {-521099, -16319743},
   {-495399, -15175797},
   {-467299, -14016022},
   {-403599, -11637114},
   {-330099, -9184684},
   {-245699, -6619551},
   {-149199, -3904806},
   {-38799, -990091},
   {117100, 2920661},
   {524800, 12802735},
   {970500, 23895984},
   {1250700, 30719361},
   {1467100, 35900951},
   {1708800, 41633726},
   {2516600, 60613122},
   {2838600, 68229597},
   {3182100, 76467347},
   {3504800, 84346050},
   {3711200, 89465637},
   {3914400, 94571026},
   {4113800, 99647162},
   {4307600, 104648324},
   {4492900, 109499150},
   {4669900, 114204326},
   {4838500, 118761147},
   {4999900, 123201957}
};

int32_t hot_junction_temperature(uint16_t cjv_adc_value,
                                 uint16_t hjv_adc_value,
                                 uint16_t intref_adc_value)
{
   const int size = sizeof(hjv2t) / sizeof(point_t);
   const int32_t r1 = 10000; // Ohm
   const int32_t r2 = 910000; // Ohm
   const int32_t rtc = 29; // Ohm
   const int32_t vref = 10; // mV
   const int32_t rvdd = 3300; // mv
   
   int32_t cjv = cold_junction_voltage(cjv_adc_value);
   int32_t vdd = rvdd * *VREFINT_CAL * 100 / intref_adc_value;
   
   int32_t vout = hjv_adc_value * vdd / 0xfff;
   int32_t vt = vref * 1000 - vout * 10/ (r2/r1);
   int32_t tcv =  (vout * 10 - vt) * rtc/ (r1+r2);
   int32_t hjv = vt - tcv;
   return linapprox(hjv2t, size, hjv * 100 + cjv);
}

/* 
 * end of file
 */

