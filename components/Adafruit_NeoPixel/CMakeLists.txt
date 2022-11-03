cmake_minimum_required(VERSION 3.15)


set(LIBRARY_SRCS
  Adafruit_NeoPixel.cpp
  esp.c
)
set(srcs ${LIBRARY_SRCS})


set(includedirs
  .
)

set(requires arduino)
# Build Adafruit_NeoPixel as an ESP-IDF component
idf_component_register(INCLUDE_DIRS ${includedirs} SRCS ${srcs} REQUIRES ${requires})


project(Adafruit_NeoPixel VERSION 1.10.4)
