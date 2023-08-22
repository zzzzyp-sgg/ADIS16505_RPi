cmake_minimum_required(VERSION 3.0)

project(ADIS16505_RPi)

set(CMAKE_BUILD_TYPE "Debug")

include_directories("/home/zyp/bcm/bcm2835-1.73/src")
link_libraries("/home/zyp/bcm/bcm2835-1.73/src/libbcm2835.a")

include_directories(${PROJECT_SOURCE_DIR}/include)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(test test.cpp)

add_executable(RecordData RecordData.cpp src/Spi.cpp)

add_executable(testADI testADI.cpp)

# add_executable(testWiringPi testWiringPi.cpp)
# target_link_libraries(testWiringPi -lwiringPi)