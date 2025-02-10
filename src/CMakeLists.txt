find_package(Python3 COMPONENTS Interpreter Development)
include_directories(${pybind11_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_SOURCE_DIR}/third_party/includes)

link_directories(${CMAKE_SOURCE_DIR}/third_party/libs)

file(GLOB WRAPPER_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)
pybind11_add_module(vlkpy ${WRAPPER_SRCS})
target_link_libraries(vlkpy PRIVATE ViewLink)

