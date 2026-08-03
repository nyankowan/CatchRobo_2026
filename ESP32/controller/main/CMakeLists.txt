set(requires "bluepad32" "btstack" "can" "procon")
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")

