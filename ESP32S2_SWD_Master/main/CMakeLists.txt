idf_component_register(SRCS "main.c" 
                    INCLUDE_DIRS "include"
                    REQUIRES server_master
                    PRIV_REQUIRES nvs_flash
                    )
