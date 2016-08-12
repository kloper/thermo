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
    def __init__(self, r0=100000, beta=3950, t0=298.15, r_sense=132000):
        self.r0 = r0 # Ohm
        self.beta = beta
        self.t0 = t0 # Kelvin
        self.r_sense = r_sense # Ohm

        self.r_inf = self.r0 * numpy.e ** -(self.beta / self.t0)

    def temp(self, v_adc):
        """
        Accept ADC value from thermistor connected to ADC_IN0.
        Return temperature in Kelvin
        """
        return self.beta / numpy.log(
            self.r_sense / self.r_inf * (float(0xfff) / v_adc - 1)
        )

    def resistance(self, temp):
        """
        Calculate resistance of a thermistor based on temperature
        """
        return self.r_inf * (numpy.e ** (self.beta/temp))

    def temp_approximation(self, adc_start=100, adc_stop=3900, epsilon=0.3):
        """
        Calculate linear approximation to temperature curve for a range of
        ADC values.
        """
        adc_values = numpy.arange(adc_start, adc_stop)
        return rdp(zip(adc_values, self.temp(adc_values)), epsilon=epsilon)

class Sense(object):
    def __init__(self,
                 vdd=3.3,
                 thermo_resistance_start=20000,
                 thermo_resistance_stop=800000):
        self.thermo_range = [thermo_resistance_start,
                             thermo_resistance_stop]
        self.vdd = vdd

    def voltage(self, sense, thermo):
        """
        Calculate voltage on thermistor/sense resistor junction based on
        their resistances
        """
        return float(sense)/(sense+thermo) * self.vdd

    def delta(self, sense):
        """
        Calculate thermistor/sense voltage range depending on
        sense resistance and required thermistor resistance ranges
        """
        return abs(self.voltage(sense, self.thermo_range[0]) - \
                   self.voltage(sense, self.thermo_range[1]))

    def optimum_sense(self, start, stop, step):
        """
        Find sense resistance in specified interval that leads to a
        maximum voltage range in thermistor/sense junction for specified
        thermistor resistance ranges.
        """
        sense_values = range(start, stop, step)
        sense = sense_values[numpy.argmax(
            [self.delta(float(sense)) for sense in sense_values]
        )]
        return (sense, [self.voltage(sense, t) for t in self.thermo_range])

#
# end of file
#
