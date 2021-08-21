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

import bbsim

# Available Players
#   PlayerOpt 
#   PlayerWi
#   PlayerWiPARDI
#   PlayerGi
#   PlayerEps
#   PlayerUcb1
#   PlayerUcbT
#   PlayerUcbTPARDI
#   PlayerUcbT_0.73_0.19 
#   PlayerUcbT_ob
#   PlayerUcbT_ob_0.73_0.19
#   PlayerTS
#   PlayerTSPARDI
#   PlayerPoker

Player      = "PlayerOpt" 
NArms       = "2"
Delay       = '5'
PriorAlf    = "1"
PriorBet    = "1"
TimeHorizon = "10"
NRuns       = "1000000" # Default in paper 1 000 000
NThrds      = "24"      # Default in paper 24

Epsilon     = "0.1" # For epsilon-greedy or ignored
GiDiscount  = "0.9" # For GI or ignored

ProbArm1    = "0.3" # Fixed arm 1 probability or ignored
ProbArm2    = "0.5" # Fixed arm 2 probability or ignored

bbsim.run(Player, NArms, PriorAlf, PriorBet, Epsilon, GiDiscount, TimeHorizon, NRuns, NThrds, Delay)

# Runs 2-arms with fixed probabilities - NOTE: No _ob player is supported and Opt algorithm defaults to Beta(1,1) priors
# bbsim.run_fixed_2arm(Player, PriorAlf, PriorBet, Epsilon, GiDiscount, TimeHorizon, NRuns, NThrds, ProbArm1, ProbArm2, Delay)
