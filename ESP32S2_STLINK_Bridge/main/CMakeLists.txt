# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES common slave master )
set(COMPONENT_PRIV_REQUIRES nvs_flash esp_wifi )

set(COMPONENT_SRCS "main.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
