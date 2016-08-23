# -*- Python -*-
"""
@file

@brief Thermocouple calculator

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

class Thermocouple(object):
    def __init__(self, r1=10e3, r2=464.5e3, rtc=29.0, vref=40e-3, vdd=3.3):
        self.r1 = r1
        self.r2 = r2
        self.rtc = rtc
        self.vref = vref
        self.vdd = vdd

    @property
    def c2(self):
        return (self.r2 / self.r1)

    @property
    def c3(self):
        return (self.r1 + self.rtc + self.r2) / self.rtc

    def it(self, vout):
        return (vout - self.vt(vout)) / (self.r1 + self.r2)

    def vt(self, vout):
        return self.vref - vout / self.c2

    def vto(self, vout):
        return self.vt(vout) - self.it(vout) * self.rtc

#
# end of file
#
