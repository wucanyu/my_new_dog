#添加头文件搜索路径
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

include_directories("../my_lib/include")
include_directories("../control/include")
include_directories("../sim_interface/include") 
include_directories("../my_lib_sim/include")
include_directories("./include")

include_directories("../third-party/eigen3")
include_directories("../third-party/qpOASES")    
include_directories("../third-party/mujoco")
include_directories("../third-party/glfw")
include_directories("../third-party/urdfdom")
include_directories("../third-party/pinocchio")

include_directories(${allInc})
#设置搜索的库目录
link_directories("../third-party/eigen3")
link_directories("/usr/local/include/osqp")
link_directories("/usr/local/include/OsqpEigen")
link_directories("../third-party/mujoco/lin_x64")
link_directories("../third-party/glfw/lin_x64")
link_directories("../third-party/qpOASES/lin_x64")
link_directories("../third-party/urdfdom")
link_directories("../third-party/pinocchio")
link_directories("../third-party/pinocchio/pinocchio")
link_directories(${allInc})

file(GLOB_RECURSE sources "src/*.cpp")
add_library(my_wbc ${sources})

#目标生成
target_link_libraries(my_wbc  
    qpOASES
    my_lib          
    my_sim   
    my_lib_sim       
)
