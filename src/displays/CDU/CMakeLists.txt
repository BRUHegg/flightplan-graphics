file(GLOB CDU_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
FILE(GLOB CDU_HDR "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")


add_library(cdu_display STATIC ${CDU_SRC} ${CDU_HDR})
target_include_directories(cdu_display INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})


if(UNIX AND NOT APPLE)
    set_property(TARGET cdu_display PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()
