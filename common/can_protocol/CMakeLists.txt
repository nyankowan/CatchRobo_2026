if(COMMAND idf_component_register)
    # ESP-IDF
    idf_component_register(INCLUDE_DIRS "inc")

else()
    # STM32など通常のCMake
    add_library(can_protocol INTERFACE)
    target_include_directories(can_protocol INTERFACE
        "${CMAKE_CURRENT_LIST_DIR}/inc"
    )

endif()