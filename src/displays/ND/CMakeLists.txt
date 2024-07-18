file(GLOB ND_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
FILE(GLOB ND_HDR "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")


add_library(nd_display STATIC ${ND_SRC} ${ND_HDR})
target_include_directories(nd_display INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})


if(UNIX AND NOT APPLE)
    set_property(TARGET nd_display PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()
