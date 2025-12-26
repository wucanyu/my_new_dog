#添加头文件搜索路径
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../third-party/eigen3")
include_directories("/usr/local/include/osqp")    
include_directories("/usr/local/include/OsqpEigen")  
include_directories("../../my_lib/include")
include_directories("../../my_lib_sim/include")
include_directories("../../my_wbc/include")
include_directories("../sim/include")
include_directories("../sim_interface/include") 
include_directories("../../third-party/mujoco")
include_directories("../../third-party/glfw")
include_directories("../../sim_interface/include")
include_directories("../../third-party/urdfdom")
include_directories("../../third-party/pinocchio")
include_directories("../../third-party/qpOASES")
include_directories(${allInc})
#设置搜索的库目录
link_directories("../../third-party/eigen3")
link_directories("/usr/local/include/osqp")
link_directories("/usr/local/include/OsqpEigen")
link_directories("../../third-party/mujoco/lin_x64")
link_directories("../../third-party/glfw/lin_x64")
link_directories("../../third-party/qpOASES/lin_x64")
link_directories("../../third-party/urdfdom")
link_directories("../../third-party/pinocchio")
link_directories("../../third-party/pinocchio/pinocchio")
link_directories(${allInc})

add_executable(DOG_SIM src/dog_sim.cpp)

#目标生成
target_link_libraries(DOG_SIM 
    my_wbc
    my_lib_sim
    glfw 
    mujoco
)

add_executable(DOG_WBC src/dog_sim_wbc.cpp)

#目标生成
target_link_libraries(DOG_WBC 
    my_wbc
    my_lib_sim
    glfw 
    mujoco
)
