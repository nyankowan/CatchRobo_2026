if(COMMAND idf_component_register)
    # ESP-IDF
    idf_component_register(INCLUDE_DIRS "inc")

else()
    # STM32など通常のCMake
    add_library(arm INTERFACE)

    target_include_directories(arm INTERFACE
        "${CMAKE_CURRENT_LIST_DIR}/inc"
    )

endif()