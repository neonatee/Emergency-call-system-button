cmake_minimum_required(VERSION 3.15)

set(LIBRARY_SRCS
    AXP202.cpp
)
set(srcs ${LIBRARY_SRCS})


set(includedirs
    "include"
)

set(requires arduino)

idf_component_register(INCLUDE_DIRS ${includedirs} SRCS ${srcs} REQUIRES ${requires})


project(AXP202 VERSION 1.0.0)

