  #include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "calc_dyn.h"
#include <string>
#include <iostream>
#include "dog_sim.h"
#include <thread>
#include <mutex>
#include "wbc_priority.h"

const double dt = 0.002;
const double dt_25Hz = 0.04;
const double dt_100Hz = 0.01;
int MPC_stance_count = 0;  

// MuJoCo load and compile model
char load_error[1000] = "Could not load binary model";
mjModel* mj_model = mj_loadXML("./../model/new_dog/meshes/scene.xml", 0, load_error, 1000);
mjData* mj_data = mj_makeData(mj_model);

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

    double startwaitingTime = 0.5;
    double startwalkingTime = 3.0;

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
            // get t and dt
            kinDynSolver.data_Read(mj_interface.Robotstate);
            kinDynSolver.computeJ_dJ();
            kinDynSolver.computeDyn();
            kinDynSolver.update_data_to_MJ_interface(mj_interface.Robotstate);

            //控制器接收数据
            dog_sim->read_data(mj_interface.Robotstate,simTime,startwalkingTime);

            // ------------- MPC ------------
            // 支撑相
			MPC_stance_count = MPC_stance_count + 1;
            if (MPC_stance_count > (dt_100Hz / dt-1)) {
                dog_sim->update_foot_forces_grf(dt_100Hz);
                MPC_stance_count = 0;
            }
            //摆动相 
            dog_sim->main_update(dt,uiController,simTime);
            dog_sim->write_data(mj_interface.Robotstate);

            // // ------------- WBC ------------
            // // WBC Calculation
            WBC_solv.dataBusRead(mj_interface.Robotstate);
            WBC_solv.computeInK_Quadruped_3DOF(mj_interface.Robotstate.fe_pos_body);
            WBC_solv.computeDdq(kinDynSolver);
            WBC_solv.computeTau();
            WBC_solv.dataBusWrite(mj_interface.Robotstate);

            std::cout << "time now :" << std::endl;
            std::cout << simTime << std::endl;
            std::cout << "--------------------------------------------------------:" << std::endl;

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

                // std::vector<double> set_tor = { 
                //                 WBC_solv.tauJointRes(0),
                //                 WBC_solv.tauJointRes(1),
                //                 WBC_solv.tauJointRes(2),

                //                 WBC_solv.tauJointRes(3),
                //                 WBC_solv.tauJointRes(4),
                //                 WBC_solv.tauJointRes(5),

                //                 WBC_solv.tauJointRes(6),
                //                 WBC_solv.tauJointRes(7),
                //                 WBC_solv.tauJointRes(8),

                //                 WBC_solv.tauJointRes(9),
                //                 WBC_solv.tauJointRes(10),
                //                 WBC_solv.tauJointRes(11)};

                mj_interface.setMotorsTorque(set_tor);       
            }
            else
            {
                std::vector<double> set_tor = { 
                                WBC_solv.tauJointRes(0),
                                WBC_solv.tauJointRes(1),
                                WBC_solv.tauJointRes(2),

                                WBC_solv.tauJointRes(3),
                                WBC_solv.tauJointRes(4),
                                WBC_solv.tauJointRes(5),

                                WBC_solv.tauJointRes(6),
                                WBC_solv.tauJointRes(7),
                                WBC_solv.tauJointRes(8),

                                WBC_solv.tauJointRes(9),
                                WBC_solv.tauJointRes(10),
                                WBC_solv.tauJointRes(11)};

                mj_interface.setMotorsTorque(set_tor);       
            }

        }

        // 更新场景显示
        uiController.updateScene();


    }

    // free visualization storage
    uiController.Close();

    // 释放 MuJoCo 资源
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);

    return 0;
}    