################################################################################
# MIT License
# 
# Copyright (c) 2021 Sebastian Pilarski and Slawomir Pilarski
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
################################################################################

from math import sqrt
from math import log as ln

class PlayerUcbT_ob_73_19:
    """Class for Player UCBT_ob"""
    c = 0.73
    d = 0.19

    def calc_ucb(self, T, s, f, p_a, p_b):
        """Calculates UCBT_ob Value ( See our paper - Section VII: Optimizing for Bernoulli )

        Args:
            T   (int): The current time + sum of all alpha and beta priors for all arms
            s   (int): Number of successes
            f   (int): Number of failures
            p_a (int): Alpha prior
            p_b (int): Beta  prior

        Returns:
            float: Value of UCBT_ob ( fed to argmax in choose(t) )
        """
        n = s + f 

        sp = s  + p_a # s'
        fp = f  + p_b # f'
        np = sp + fp  # n'

        V = sp*fp/( np*np*(np+1) ) + sqrt( 2*ln(T)/np )

        return sp/np + self.c*sqrt( ln(T)/np * min(self.d, V) )
