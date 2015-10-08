cmake_minimum_required(VERSION 2.8.3)

project(ServoController)

include (CheckIncludeFiles)
include (CheckLibraryExists)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules)

# setup include directories

option(BUILD_SHARED_LIBS "Build as a shared library?" ON)

# fix the C++ linking error on 64-bit Linux
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

include_directories(
                    include
                    /usr/include
                    /usr/local/include
                    )

link_directories(
                  /usr/lib
                  /usr/local/lib
                  )

file(GLOB LIBSOURCES
"*.cpp"
)

add_library(ServoController ${LIBSOURCES})
#target_link_libraries(ServoController arduino-bioloid)
set_target_properties(ServoController PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS ServoController DESTINATION lib)
add_executable(ServoController-Test ${LIBSOURCES})

# install include dirs
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ServoController DESTINATION include)
