cmake_minimum_required(VERSION 3.21)
project(SPP)

set(CMAKE_CXX_STANDARD 14)
link_libraries(ws2_32)
include_directories(include)
add_executable(SPP
        main.cpp
        CMatrix.h   CMatrix.cpp
        lib.h
        TimeConv.cpp
        CoorConv.cpp
        CDecode.h   CDecode.cpp
        SatPositioning.hpp  SatPositioning.cpp
        CDetectOutlier.h    CDetectOutlier.cpp
        SPP.h   SPP.cpp
        Client.h    Client.cpp)
