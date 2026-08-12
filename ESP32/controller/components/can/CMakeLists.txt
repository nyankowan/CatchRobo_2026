set(requires "driver" "can_protocol")
idf_component_register(SRCS "can.c"
                    INCLUDE_DIRS "."
                    REQUIRES ${requires})