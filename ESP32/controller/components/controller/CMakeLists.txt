set(requires "bluepad32" "btstack")
idf_component_register(SRCS "my_platform.c"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")