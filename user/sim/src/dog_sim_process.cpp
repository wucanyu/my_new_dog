#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "calc_dyn.h"
#include <string>
#include <iostream>
#include "dog_sim.h"
#include <sched.h>
#include <pthread.h>
#include <thread>
#include <mutex>
#include "wbc_priority.h"

const double dt = 0.002;
const double dt_100Hz = 0.01;
int MPC_stance_count = 0;  

//条件变量
std::condition_variable cv; 
// 全局共享资源 互斥量
std::mutex mpc_mutex;
// 原子退出标志：保证多线程内存可见性，用于终止MPC线程
std::atomic<bool> sim_running(true);

// MuJoCo load and compile model
char load_error[1000] = "Could not load binary model";
mjModel* mj_model = mj_loadXML("./../model/new_dog/meshes/scene.xml", 0, load_error, 1000);
mjData* mj_data = mj_makeData(mj_model);

// pthread_t thread: 要设置亲和性的线程ID。
// size_t cpusetsize: cpuset的大小，通常为sizeof(cpu_set_t)。
// cpu_set_t *cpuset: 一个CPU集合，用于指定线程可以运行的CPU核心。
int set_cpu_affinity(pthread_t thread, int cpu_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);

    int result = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (result != 0) {
        std::cerr << "Failed to set CPU affinity: " << result << std::endl;
        return -1;
    }
    return 0;
}

int main(int argc, char **argv) {
    UIctr uiController(mj_model, mj_data);   // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco
    Pin_KinDyn kinDynSolver("./../model/new_dog/meshes/new_dog.urdf"); // kinematics and dynamics solver
    WBC_priority WBC_solv(kinDynSolver.model_nv, 18, 22, 0.4, mj_model->opt.timestep); // WBC solver
    uiController.iniGLFW();
    uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo", false); 
    mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq*1); // set ini pos in Mujoco

    // create controller
    std::unique_ptr<HardwareDog> dog_sim = std::make_unique<HardwareDog>();
    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;

    double startwaitingTime = 0.2;
    double startwalkingTime = 3.0;

    // -------------------------- 1. MPC计算线程（独立线程） --------------------------
    std::thread compute_MPC_thread([&]() {
        while (sim_running.load(std::memory_order_acquire)) {
            bool need_mpc = false;
            // 步骤1：加锁检测MPC触发条件（读取共享变量MPC_stance_count）
            {
                std::lock_guard<std::mutex> lock(mpc_mutex); // 自动加锁，作用域结束自动解锁
                if (MPC_stance_count > (dt_100Hz / dt - 1)) {
                    need_mpc = true;
                }
            } // 锁在此处自动释放，避免MPC计算期间持有锁

            // 步骤2：执行耗时MPC计算（无需持有锁，仅在读写共享资源时加锁）
            if (need_mpc) {
                std::lock_guard<std::mutex> lock(mpc_mutex);
                dog_sim->update_foot_forces_grf(dt_100Hz); // 访问共享资源dog_sim，加锁保护
                MPC_stance_count = 0; // 重置共享计数，加锁保护
            }
            // 步骤3：精准延时，避免CPU空转（与仿真步长一致）
            std::this_thread::sleep_for(std::chrono::duration<double>(dt));
        }
    });

    // -------------------------- 设置 MPC 线程 CPU 亲和度（关键修改1） --------------------------
    // 1. 获取 MPC 线程的 pthread_t 句柄
    pthread_t mpc_thread_handle = compute_MPC_thread.native_handle();
    // 2. 绑定到 CPU 核心 0（可根据你的CPU空闲情况修改，如 1、2 等）
    int mpc_cpu_id = 0;
    if (set_cpu_affinity(mpc_thread_handle, mpc_cpu_id) == 0) {
        std::cout << "MPC thread bound to CPU core " << mpc_cpu_id << std::endl;
    }

    // -------------------------- 2. 主仿真线程 --------------------------
    std::thread main_thread([&]() {
        while (!glfwWindowShouldClose(uiController.window)) {
            simstart = mj_data->time;
            // press "1" to pause and resume, "2" to step the simulation
            while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim) {
                // 推进仿真
                mj_step(mj_model, mj_data);
                simTime=mj_data->time;
                
                // 更新传感器值
                mj_interface.updateSensorValues();
                mj_interface.update_data_to_phnocchio();

                // update kinematics and dynamics info
                kinDynSolver.data_Read(mj_interface.Robotstate);
                kinDynSolver.computeJ_dJ();
                kinDynSolver.computeDyn();
                kinDynSolver.update_data_to_MJ_interface(mj_interface.Robotstate);
                //控制器接收数据
                dog_sim->read_data(mj_interface.Robotstate,simTime,startwalkingTime);

                //支撑相 加锁自增共享变量MPC_stance_count（保护共享资源）
                {
                    std::lock_guard<std::mutex> lock(mpc_mutex);
                    MPC_stance_count += 1;
                }
            
                std::cout << "time now :" << std::endl;
                std::cout << simTime << std::endl;
                //摆动相 
                dog_sim->main_update(dt,uiController,simTime);

                if(simTime <= startwaitingTime)
                {
                    std::vector<double> set_tor = {0,0,0,0,0,0,0,0,0,0,0,0};
                    mj_interface.setMotorsTorque(set_tor);                          
                }
                else if(simTime <= startwalkingTime)
                {
                    std::vector<double> set_tor = { 
                                    dog_sim->ctrl_states.joint_torques_out(0,0),
                                    dog_sim->ctrl_states.joint_torques_out(1,0),
                                    dog_sim->ctrl_states.joint_torques_out(2,0),
                                    dog_sim->ctrl_states.joint_torques_out(3,0),
                                    dog_sim->ctrl_states.joint_torques_out(4,0),
                                    dog_sim->ctrl_states.joint_torques_out(5,0),
                                    dog_sim->ctrl_states.joint_torques_out(6,0),
                                    dog_sim->ctrl_states.joint_torques_out(7,0),
                                    dog_sim->ctrl_states.joint_torques_out(8,0),
                                    dog_sim->ctrl_states.joint_torques_out(9,0),
                                    dog_sim->ctrl_states.joint_torques_out(10,0),
                                    dog_sim->ctrl_states.joint_torques_out(11,0)};
                    mj_interface.setMotorsTorque(set_tor);       
                }
                else
                {
                    std::vector<double> set_tor = { 
                                    dog_sim->ctrl_states.joint_torques_out(0,0),
                                    dog_sim->ctrl_states.joint_torques_out(1,0),
                                    dog_sim->ctrl_states.joint_torques_out(2,0),
                                    dog_sim->ctrl_states.joint_torques_out(3,0),
                                    dog_sim->ctrl_states.joint_torques_out(4,0),
                                    dog_sim->ctrl_states.joint_torques_out(5,0),
                                    dog_sim->ctrl_states.joint_torques_out(6,0),
                                    dog_sim->ctrl_states.joint_torques_out(7,0),
                                    dog_sim->ctrl_states.joint_torques_out(8,0),
                                    dog_sim->ctrl_states.joint_torques_out(9,0),
                                    dog_sim->ctrl_states.joint_torques_out(10,0),
                                    dog_sim->ctrl_states.joint_torques_out(11,0)};
                    mj_interface.setMotorsTorque(set_tor);       
                }

            }
            // 更新场景显示
            uiController.updateScene();
        }

        //设置退出标志，终止MPC线程
        sim_running.store(false, std::memory_order_release);
        // free visualization storage
        uiController.Close();

        // 释放 MuJoCo 资源
        mj_deleteData(mj_data);
        mj_deleteModel(mj_model);
    });

    // -------------------------- 设置 主仿真线程 CPU 亲和度（关键修改2） --------------------------
    // 1. 获取主仿真线程的 pthread_t 句柄
    pthread_t main_thread_handle = main_thread.native_handle();
    // 2. 绑定到 CPU 核心 1（与 MPC 线程不同核心，避免竞争）
    int main_cpu_id = 1;
    if (set_cpu_affinity(main_thread_handle, main_cpu_id) == 0) {
        std::cout << "Main simulation thread bound to CPU core " << main_cpu_id << std::endl;
    }

    // -------------------------- 3. 等待两个线程正常退出 --------------------------
    compute_MPC_thread.join();
    main_thread.join();

    return 0;
}