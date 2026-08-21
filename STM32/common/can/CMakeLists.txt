add_library(stm_can INTERFACE)

target_sources(stm_can INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/src/stm_can.c"
)

target_include_directories(stm_can INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/inc"
    "${CMAKE_CURRENT_LIST_DIR}/../../../common/can_protocol/inc"
)