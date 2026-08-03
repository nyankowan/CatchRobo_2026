set(requires "bluepad32" "btstack")
idf_component_register(SRCS "my_platform.c" "procon_data.c" "procon_data"
                    INCLUDE_DIRS "."
                    REQUIRES ${requires})