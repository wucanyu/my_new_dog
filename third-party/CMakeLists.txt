# 设置库的路径
set(dirEigen 	"./eigen3")
set(dirGlfw     "./glfw")
set(libURDF     "./urdfdom")
set(incMujoco   "./mujoco")
set(dirPino   "./pinocchio")
set(incQP 		"./qpOASES")
# 初始化所有库的列表
set(allLib ${dirGlfw}  ${incMujoco} ${libURDF} ${dirPino} ${dirEigen})
set(allInc ${allLib} ${incQP})

# 根据处理器架构添加不同的 Mujoco 库路径
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    list(APPEND allLib "${incMujoco}/lin_arm64")
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "AMD64")
    list(APPEND allLib "${incMujoco}/lin_x64")
else()
    message(FATAL_ERROR "Unsupported processor architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()


include_directories(${allInc})

# 设置库文件搜索路径
link_directories(${allLib})






