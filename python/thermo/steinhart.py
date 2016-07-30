# -*- Python -*-
"""
@file

@brief Calculate Steinhart-Hart lookup table for thermistor

@page License

Copyright (c) 2016, Dimitry Kloper.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

* Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""

import numpy

# Ramer-Douglas-Peucker Algorithm implementation from
# git@github.com:fhirschmann/rdp.git
from rdp import rdp

class Thermistor(object):
    def __init__(self):
        self.r0 = 100000 # Ohm
        self.beta = 3950
        self.t0 = 298.15 # Kelvin
        self.r_sense = 300000 # Ohm

        self.r_inf = self.r0 * numpy.e ** -(self.beta / self.t0)

    def temp(self, v_adc):
        return self.beta / numpy.log(
            self.r_sense / self.r_inf * (float(0xfff) / v_adc - 1)
        )

    def approximation(self):
        adc_values = numpy.arange(100, 3900)
        return rdp(zip(adc_values, self.temp(adc_values)), epsilon=0.3)

#
# end of file
#
