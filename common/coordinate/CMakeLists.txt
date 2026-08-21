if(COMMAND idf_component_register)
    # ESP-IDF
    idf_component_register(SRC_DIRS "src"
                    INCLUDE_DIRS "inc")

else()
    # STM32など通常のCMake
    add_library(coordinate INTERFACE)
    target_sources(coordinate INTERFACE
        "${CMAKE_CURRENT_LIST_DIR}/src/coordinate.c"
    )
    target_include_directories(coordinate INTERFACE
        "${CMAKE_CURRENT_LIST_DIR}/inc"
    )

endif()