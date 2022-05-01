cmake_minimum_required(VERSION 3.21)
project(RTK)

set(CMAKE_CXX_STANDARD 14)
link_libraries(ws2_32)
include_directories(include)
add_executable(RTK
        main.cpp
        RTKLib.h CMatrix.cpp TimeConv.cpp CoorConv.cpp
        CDecode.h   CDecode.cpp
        SatPos.h    SatPos.cpp
        Detect.h Detect.cpp
        SPP.h   SPP.cpp
        Client.h    Client.cpp
        lambda.cpp
        RTK.cpp RTK.h SDObs.cpp SDObs.h)
