# **Optimal Policy for Bernoulli Bandits**
Provides various classes to (multi-thread) compute: 
* Optimal policy for k-arms, time horizon, and Beta prior
* Efficient implementation of indexing scheme including its correctness test
* Configurations iterator
* Expected value per time horizon and Beta prior
* Reachability 

See [OptPol.cpp](OptPol.cpp) for C++ implementation.

See [OptPol.py](OptPol.py) for Python API examples.

See [build.me](build.me) for CMAKE build script.

## **Requirements**
Currently, Python APIs require Boost.Python (libboost_python) and Linux (x86-64 CPU architecture).