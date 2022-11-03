set(min_supported_idf_version "4.1.0")
set(max_supported_idf_version "4.4.99")
set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}.${IDF_VERSION_PATCH}")

set(LIBRARY_SRCS
  src/Module.cpp
  src/protocols/PhysicalLayer/PhysicalLayer.cpp
  src/modules/SX126x/SX126x.cpp
  src/modules/SX126x/SX1261.cpp
  src/modules/SX126x/SX1262.cpp
  src/modules/SX126x/SX1268.cpp
  src/modules/SX127x/SX127x.cpp
  src/modules/SX127x/SX1272.cpp
  src/modules/SX127x/SX1273.cpp
  src/modules/SX127x/SX1276.cpp
  src/modules/SX127x/SX1277.cpp
  src/modules/SX127x/SX1278.cpp
  src/modules/SX127x/SX1279.cpp
)

set(includedirs
  src
)

set(srcs ${LIBRARY_SRCS})
set(requires arduino)
idf_component_register(INCLUDE_DIRS ${includedirs} SRCS ${srcs} REQUIRES ${requires})
project(RadioLib VERSION 5.1.2)

#target_compile_options(${COMPONENT_TARGET} PUBLIC
 #   -DESP32)

