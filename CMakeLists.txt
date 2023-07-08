cmake_minimum_required(VERSION 3.12)

project(control_system) # 项目名称

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS true)
set(CMAKE_C_STANDARD_REQUIRED True)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS true)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set(OpenCV_STATIC ON)
# find_package(OpenCV REQUIRED)

find_package(Threads REQUIRED)

# 可执行文件输出路径
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

file(GLOB_RECURSE SOURCES
    src/*.cpp
    src/*.hpp
    src/*.c
    src/*.h
)

add_executable(${PROJECT_NAME}
    ${SOURCES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    .
    src
    # ${OpenCV_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
)

target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    # 对 MSVC 开启 utf-8 支持
    "$<$<C_COMPILER_ID:MSVC>:/source-charset:utf-8>"
    "$<$<CXX_COMPILER_ID:MSVC>:/source-charset:utf-8>"
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    m
    # ${OpenCV_LIBS} gomp
    ${CMAKE_THREAD_LIBS_INIT}
)

target_link_options(${PROJECT_NAME} PRIVATE
    # -static
)