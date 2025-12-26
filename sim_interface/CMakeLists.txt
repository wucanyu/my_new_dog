#添加头文件搜索路径
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("../third-party/eigen3") 
include_directories("../sim_interface/include") 
include_directories("../third-party/mujoco")
include_directories("../third-party/glfw")

file(GLOB_RECURSE sources
"src/*.cpp"
)

# 将变量的cpp文件生成共享Library
add_library(my_sim ${sources})        

#目标生成
target_link_libraries(my_sim my_lib)