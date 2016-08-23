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
   {400, -87607},
   {454, -78964},
   {513, -70306},
   {576, -61773},
   {644, -53219},
   {718, -44528},
   {797, -35817},
   {882, -26966},
   {974, -17872},
   {1052, -10480},
   {1135, -2872},
   {1225, 5133},
   {1323, 13627},
   {1509, 29306},
   {1981, 68385},
   {2112, 79489},
   {2229, 89648},
   {2351, 100570},
   {2463, 110977},
   {2568, 121146},
   {2667, 131179},
   {2762, 141296},
   {2851, 151295},
   {2935, 161282},
   {3015, 171382},
   {3090, 181467},
   {3161, 191666},
   {3228, 201977},
   {3292, 212564},
   {3332, 219607},
   {3370, 226644},
   {3407, 233861},
   {3443, 241270},
   {3477, 248662},
   {3510, 256249},
   {3542, 264044},
   {3572, 271795},
   {3601, 279748},
   {3629, 287914},
   {3656, 296303},
   {3681, 304582},
   {3706, 313423},
   {3729, 322123},
   {3752, 331444},
   {3773, 340577},
   {3793, 349911},
   {3812, 359437},
   {3830, 369144},
   {3839, 374279},
   {3848, 379620},
   {3857, 385184},
   {3865, 390333},
   {3873, 395687},
   {3881, 401264},
   {3889, 407082},
   {3896, 412386},
   {3903, 417907},
   {3910, 423660},
   {3917, 429667},
   {3923, 435034},
   {3930, 441571},
   {3936, 447431},
   {3942, 453550},
   {3948, 459950},
   {3954, 466656},
   {3960, 473697},
   {3966, 481107},
   {3971, 487592},
   {3976, 494384},
   {3981, 501514},
   {3986, 509014},
   {3990, 515306},
   {3995, 523573},
   {3999, 530539}
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
   int32_t vdd = (3300 * *VREFINT_CAL / intref_adc_value) * 10000;
   int32_t vout = hjv_adc_value * (vdd / 0xfff) + 50000;
   int32_t cjv = cold_junction_voltage(cjv_adc_value);
   int32_t vt = 400000 - (vout*100) / 4645;
   int32_t it =  (vout - vt) / 47450;
   int32_t hjv = vt * 10 - it * 29;
   const int size = sizeof(hjv2t) / sizeof(point_t);
   return linapprox(hjv2t, size, hjv + cjv);
}

/* 
 * end of file
 */

