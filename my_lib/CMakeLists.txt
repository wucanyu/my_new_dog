#添加头文件搜索路径
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("./include")
include_directories("../third-party/")
include_directories("../third-party/qpOASES/include")
include_directories("/usr/local/include/eigen3") 
include_directories("/usr/local/include/osqp")    
include_directories("/usr/local/include/OsqpEigen")  
include_directories("../my_lib/include") 
include_directories("../user/mit_kf/include") 
#设置搜索的库目录
link_directories("/usr/local/include/eigen3")
link_directories("/usr/local/include/OsqpEigen")
link_directories("/usr/local/include/osqp")

# 查找 Boost 库，确保包含文件系统和系统组件
find_package(Boost REQUIRED COMPONENTS filesystem system )

file(GLOB_RECURSE sources
"src/*.cpp"
)
# 将变量的cpp文件生成共享Library
add_library(my_lib ${sources})        

#目标生成
target_link_libraries(my_lib  pthread Boost::filesystem Boost::system OsqpEigen osqp)