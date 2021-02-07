cmake_minimum_required(VERSION 3.0)

set( Python_INCLUDE_DIR /usr/include/python2.7/ ) # Note: make sure it matches your distro

project(OptPol)
FIND_PACKAGE(PythonInterp)
FIND_PACKAGE(PythonLibs)
FIND_PACKAGE(Boost COMPONENTS python)

INCLUDE_DIRECTORIES( ${Python_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

PYTHON_ADD_MODULE(OptPol OptPol.cpp)
TARGET_LINK_LIBRARIES( OptPol boost_python )

#FILE(COPY MyProject.py DESTINATION .) # See the whole tutorial to understand this line

