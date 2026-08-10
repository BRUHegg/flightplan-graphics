file(GLOB_RECURSE UTIL_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(util_lib STATIC ${UTIL_SRC})
target_compile_options(util_lib PUBLIC -g)

if(UNIX AND NOT APPLE)
    set_property(TARGET util_lib PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()