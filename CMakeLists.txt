cmake_minimum_required(VERSION 3.5)

project(my_dog)

add_compile_options(-std=c++17)

#禁止在源代码目录中构建项目
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
#禁止CMake在构建过程中修改源代码目录
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)
#前面实现检查
if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()
#颜色高亮
set(CMAKE_COLOR_MAKEFILE   ON)
#在查找自定义模块时，应该首先在源代码根目录下的cmake目录中查找
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_subdirectory(sim_interface)
add_subdirectory(third-party)
add_subdirectory(my_lib)
add_subdirectory(my_lib_sim)
add_subdirectory(my_wbc)
add_subdirectory(user)

