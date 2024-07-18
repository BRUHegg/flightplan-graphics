cmake_minimum_required(VERSION 3.13)
project(fpln_graphics CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -fPIC -g")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory("src/fpln")
add_subdirectory(src/displays)

file(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
FILE(GLOB HDR_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp")
add_executable(fpln_graphics ${SRC_FILES} ${HDR_FILES})

set_target_properties(fpln_graphics PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}")

if(WIN32)
	add_definitions(-DAPL=0 -DIBM=1 -DLIN=0)
    add_definitions(-D_WIN32_WINNT=0x0600)
    set(ARCH "mingw64")
    
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -static-libgcc")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libgcc -static-libstdc++")
    set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_C_FLAGS} -static-libgcc -s")
    set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS}  -static-libgcc -static-libstdc++ -s")
else()
    set(ARCH "lin64")
endif()

include(CMakePrintHelpers)
option(LIBNAV "/home/betatest/Documents/programming/libnav-redist" "")
cmake_print_variables(LIBNAV)
target_include_directories(fpln_graphics PUBLIC "${LIBNAV}/include")
target_include_directories(fpln PUBLIC "${LIBNAV}/include")
file(GLOB LIBNAV_LIBS ${LIBNAV}/${ARCH}/*.a)
cmake_print_variables(LIBNAV_LIBS)
target_link_libraries(fpln PUBLIC ${LIBNAV_LIBS})
target_link_libraries(nd_display PUBLIC fpln)
#target_link_libraries(fpln_graphics PUBLIC )

# Configure gtk

find_package (PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

include_directories(${GTK3_INCLUDE_DIRS})
link_directories(${GTK3_LIBRARY_DIRS})
add_definitions(${GTK3_CFLAGS_OTHER})
target_link_libraries(nd_display PUBLIC ${GTK3_LIBRARIES})
target_link_libraries(fpln_graphics PUBLIC ${GTK3_LIBRARIES} fpln nd_display)

target_include_directories(nd_display PUBLIC "${LIBNAV}/include" "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/src/displays" ${GTK3_INCLUDE_DIRS})

# Configure freetype

find_package(Freetype REQUIRED)
target_link_libraries(fpln_graphics PUBLIC ${FREETYPE_LIBRARIES})
target_include_directories(fpln_graphics PUBLIC ${FREETYPE_INCLUDE_DIRS})
