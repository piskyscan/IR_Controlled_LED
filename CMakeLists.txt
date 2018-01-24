cmake_minimum_required (VERSION 2.6)

project (IR_Controlled_LED)

find_library(CARMP3LIB carmp3lib)
find_library(WS2811 ws2811)

add_executable(IR_Controlled_LED IR_Controlled_LED.c dma.c rpihw.c pwm.c pcm.c ws2811.c)

target_link_libraries(IR_Controlled_LED  "${CARMP3LIB}" "${WS2811}" pigpio rt)
