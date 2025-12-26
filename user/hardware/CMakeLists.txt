#添加头文件搜索路径
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("/usr/local/include/eigen3") 
include_directories("/usr/local/include/osqp")    
include_directories("/usr/local/include/OsqpEigen")  
include_directories("../../my_lib/include")
include_directories("../hardware/include")
include_directories(${allInc})
#设置搜索的库目录
link_directories("/usr/local/include/eigen3")
link_directories("/usr/local/include/osqp")
link_directories("/usr/local/include/OsqpEigen")
# 查找 Boost 库，确保包含文件系统和系统组件

file(GLOB_RECURSE sources
"src/*.cpp"
)

add_executable(DOG ${sources})
#目标生成
target_link_libraries(DOG my_lib)

