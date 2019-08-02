cmake_minimum_required(VERSION 3.5)
project(untitled C CXX)

#set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
#set(ALT_DEVICE_FAMILY "soc_cv_av")
#
#set(CMAKE_C_FLAGS "${ALT_DEVICE_FAMILY}")
#set(CMAKE_CXX_FLAGS "${ALT_DEVICE_FAMILY}")
#

#add_library(PIDlib SHARED IMPORTED)
#set_property(TARGET PIDlib PROPERTY IMPORTED_LOCATION "/usr/include/rc/math")

SET(ALT_DEVICE_FAMILY soc_cv_av)
add_definitions(-D${ALT_DEVICE_FAMILY})

add_executable(untitled main.cpp main.h motorController.cpp motorController.h fpgaAddressComm.h fpgaEncoder.h)
target_link_libraries(untitled "/usr/lib/librobotcontrol.so")








