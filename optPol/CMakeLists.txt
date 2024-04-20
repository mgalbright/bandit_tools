cmake_minimum_required(VERSION 3.14)

project(OptPol)
find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)

include_directories( ${Python_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

python_add_module(OptPol OptPol.cpp)
target_link_libraries( OptPol ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

#FILE(COPY MyProject.py DESTINATION .) # See the whole tutorial to understand this line

