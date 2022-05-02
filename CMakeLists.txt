cmake_minimum_required(VERSION 2.8)
set(SOLUTIONTITLE Reeb-Graph CACHE TYPE STRING)
project(${SOLUTIONTITLE})

file(GLOB FILES_SRC
    "src/*.cpp"
    "src/*.h"
  
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(VVRFRAMEWORK_DIR "" CACHE PATH "Location of VVR Framework")


include_directories(${CMAKE_SOURCE_DIR})
include_directories(/usr/local/include/VVRScene)
include_directories(/usr/local/include/GeoLib)
include_directories(/usr/local/include/MathGeoLib)
 

add_executable(${SOLUTIONTITLE} ${FILES_SRC})

target_link_libraries(
        ${SOLUTIONTITLE}
        -lVVRScene
        -lGeoLib
        -lMathGeoLib

)
 

