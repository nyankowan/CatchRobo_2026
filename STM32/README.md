
# 各プロジェクトのCMakeLists.txt
```CMake
# STM32共通CAN
add_subdirectory(
    "${CMAKE_CURRENT_LIST_DIR}/../common/can"
    "${CMAKE_BINARY_DIR}/stm_can"
)

# ESP32 / STM32共通 CAN protocol
add_subdirectory(
    "${CMAKE_CURRENT_LIST_DIR}/../../common/can_protocol"
    "${CMAKE_BINARY_DIR}/can_protocol"
)

# ESP32 / STM32共通 座標処理
add_subdirectory(
    "${CMAKE_CURRENT_LIST_DIR}/../../common/coordinate"
    "${CMAKE_BINARY_DIR}/coordinate"
)



target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
    stm_can
    can_protocol
    coordinate
)
```