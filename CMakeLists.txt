project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources
        src/main.cpp
        src/TrajectoryGenerator.cpp
        src/Vehicle.cpp
        src/helpers.cpp
        src/trajectory_tools.cpp
        src/state/State.cpp
        src/state/KeepLane.cpp
        src/state/PLCL.cpp src/state/PLC.cpp src/state/PLC.h src/state/LCL.cpp src/state/LCL.h src/state/LC.cpp src/state/LC.h src/state/PLCR.cpp src/state/PLCR.h src/state/LCR.cpp src/state/LCR.h)

include_directories(src)
include_directories(src/state)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
