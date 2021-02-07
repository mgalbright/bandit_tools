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

import OptPol

# number of threads to use
nthrds=6

# arms and horizon
k=2    #number of arms
H=12   # time horizon

# prior Beta parameters, Beta(alf,bet)
alf=1
bet=1

# run test to check if consecutive configurations have consecutive index values
# NOTE: grep for "Error"
#OptPol.test(k,H,1) # nthreads = 1

# compute k-arm H-time_horizon normalized expected reward
#OptPol.expected(k,H,alf,bet,nthrds)


# compute k-arm H-time_horizon reachability stats
#OptPol.reachable(k,t,alf,bet,nthrds)

# 2-arms: get configuration index (location in array)
# NOTE: call anytime
# print (OptPol.index2(s0,f0,s1,f1))

# 3-arms: get configuration index (location in array)
# NOTE: call anytime
# print (OptPol.index3(s0,f0,s1,f1,s2,f2))

# compute k-arm H-time_horizon Beta(alf,bet)-prior optimal policy
#OptPol.compute(k,H,alf,bet,nthrds)

# check current optimal policy parameters
#print (OptPol.n_arms())
#print (OptPol.alf())
#print (OptPol.bet())

# 2-arm success/failure stats per arm (configurations)
s0=1
f0=1
s1=1
f1=1

# 2-arms: get best arm for a configuration
# NOTE: use after OptPol.compute(2,...) was run first
# print (OptPol.best_arm2(s0,f0,s1,f1))

# 3-arm success/failure stats per arm (configurations)
# NOTE: it extends 2-arm stats
s2=1
f2=1

# 3-arms: get best arm for a configuration
# NOTE: use after OptPol.compute(3,...) was run first
# print (OptPol.best_arm3(s0,f0,s1,f1,s2,f2))
