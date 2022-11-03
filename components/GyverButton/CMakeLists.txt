set(LIBRARY_SRCS
    src/GyverButton.cpp
)
set(srcs ${LIBRARY_SRCS})


set(includedirs
    "src"
)

set(requires arduino)

idf_component_register(INCLUDE_DIRS ${includedirs} SRCS ${srcs} REQUIRES ${requires})


project(GyverButton VERSION 3.8)


