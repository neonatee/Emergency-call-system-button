
set(LIBRARY_SRCS
main.cpp
)
set(srcs ${LIBRARY_SRCS})


set(includedirs
    "include"
)

set(requires db-logger device wifi-manager power-manager lora-radio button pixel-manager)


idf_component_register(INCLUDE_DIRS ${includedirs} SRCS ${srcs} REQUIRES ${requires})
spiffs_create_partition_image(spiffs ../spiffs_image FLASH_IN_PROJECT)


