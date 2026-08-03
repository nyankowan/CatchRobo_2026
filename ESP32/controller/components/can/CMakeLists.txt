set(requires "driver")
idf_component_register(SRCS "can.c"
                    INCLUDE_DIRS "."
                    REQUIRES ${requires})