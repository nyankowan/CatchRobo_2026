set(requires "bluepad32" "btstack")
idf_component_register(SRCS "main.c" "my_platform.c"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")