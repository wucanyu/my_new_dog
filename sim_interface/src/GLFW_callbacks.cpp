#include "GLFW_callbacks.h"

UIctr::UIctr(mjModel *modelIn, mjData *dataIn) {
    mj_model=modelIn;  // 初始化模型指针
    mj_data=dataIn;    // 初始化数据指针
    cam=mjvCamera();   // 初始化相机
    opt=mjvOption();   // 初始化可视化选项
    scn=mjvScene();    // 初始化场景
    con=mjrContext();   // 初始化渲染上下文
}

// GLFW滚动回调，转发到类成员函数
static void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Scroll(xoffset, yoffset);
}
// 鼠标移动回调
static void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Mouse_move(xpos, ypos);
}
// 鼠标按键回调
static void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Mouse_button(button, act, mods);
}
// 键盘回调
static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Keyboard(key, scancode, act, mods);
}
// 窗口关闭回调
static void window_close_callback(GLFWwindow* window)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Close();
}

void UIctr::iniGLFW() {
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

}

// 创建窗口及相关设置
void UIctr::createWindow(const char* windowTitle, bool saveVideo) {
    // 创建 GLFW 窗口
    window = glfwCreateWindow(width, height, windowTitle, NULL, NULL);
    glfwMakeContextCurrent(window);  // 设置当前窗口的 OpenGL 上下文
    glfwSwapInterval(1);  // 启用垂直同步

    // 初始化相机参数（观察视角）
    double arr_view[] = {150, -16, 3, 0, 0.000000, 1.00000}; 
    cam.azimuth = arr_view[0];    // 方位角
    cam.elevation = arr_view[1];  // 仰角
    cam.distance = arr_view[2];   // 相机到目标的距离
    cam.lookat[0] = arr_view[3];  // 观察点 X 坐标
    cam.lookat[1] = arr_view[4];  // 观察点 Y 坐标
    cam.lookat[2] = arr_view[5];  // 观察点 Z 坐标

    if (isTrack) {  // 启用跟踪相机模式
        cam.lookat[2] += 0.8;
        cam.type = mjCAMERA_TRACKING;  // 跟踪模式
        cam.trackbodyid = 1;  // 跟踪的物体 ID
    } else
        cam.type = mjCAMERA_FREE;  // 自由相机模式

    // 初始化 MuJoCo 可视化对象
    mjv_defaultOption(&opt);  // 默认可视化选项
    mjv_defaultScene(&scn);   // 默认场景
    mjr_defaultContext(&con); // 默认渲染上下文
    mjv_makeScene(mj_model, &scn, 2000);  // 创建场景（容纳 2000 个物体）
    mjr_makeContext(mj_model, &con, mjFONTSCALE_150);  // 创建渲染上下文

    // 设置 GLFW 回调函数
    glfwSetWindowUserPointer(window, this);  // 将窗口与当前对象绑定
    glfwSetWindowCloseCallback(window, window_close_callback);  // 窗口关闭回调
    glfwSetKeyCallback(window, keyboard);  // 键盘事件回调
    glfwSetCursorPosCallback(window, mouse_move);  // 鼠标移动回调
    glfwSetMouseButtonCallback(window, mouse_button);  // 鼠标按键回调
    glfwSetScrollCallback(window, scroll);  // 滚动回调

    save_video = saveVideo;  // 记录是否保存视频
}

// 更新场景（渲染、处理事件）
void UIctr::updateScene() {
    if (!isContinuous)
        runSim = false;  // 根据模式设置仿真运行状态

    // 重置按键状态（避免重复触发）
    buttonRead.key_w = false;
    buttonRead.key_a = false;
    buttonRead.key_s = false;
    buttonRead.key_d = false;
    buttonRead.key_space = false;
    buttonRead.key_h = false;
    buttonRead.key_j = false;

    // 获取窗口帧缓冲区尺寸
    mjrRect viewport = {0, 0, 0, 0};
    glfwMakeContextCurrent(window);
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // 更新场景并渲染
    mjv_updateScene(mj_model, mj_data, &opt, NULL, &cam, mjCAT_ALL, &scn);  // 更新场景
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjr_render(viewport, &scn, &con);  // 渲染到视口

    // 显示仿真时间
    char buffer[100];
    std::sprintf(buffer, "Time: %.3f", mj_data->time);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, buffer, NULL, &con);

    glfwSwapBuffers(window);  // 交换前后缓冲区（显示渲染结果）
    glfwPollEvents();  // 处理 GLFW 事件（如按键、鼠标移动）
}



// 键盘事件处理
void UIctr::Keyboard(int key, int scancode, int act, int mods) {
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {  // 退格键：重置仿真
        mj_resetData(mj_model, mj_data);  // 重置模型数据
        mj_forward(mj_model, mj_data);  // 推进仿真
    }

    if (act == GLFW_RELEASE && key == GLFW_KEY_1) {  // 1 键：切换运行状态
        runSim = !runSim;
        isContinuous = true;
    }

    if (act == GLFW_RELEASE && key == GLFW_KEY_2) {  // 2 键：设置非连续模式
        runSim = true;
        isContinuous = false;
    }


    if (act == GLFW_RELEASE && key == GLFW_KEY_W) buttonRead.key_w = true;
    if (act == GLFW_RELEASE && key == GLFW_KEY_A) buttonRead.key_a = true;
    if (act == GLFW_RELEASE && key == GLFW_KEY_S) buttonRead.key_s = true;
    if (act == GLFW_RELEASE && key == GLFW_KEY_D) buttonRead.key_d = true;
    if (act == GLFW_RELEASE && key == GLFW_KEY_H) buttonRead.key_h = true;
    if (act == GLFW_RELEASE && key == GLFW_KEY_J) buttonRead.key_j = true;
    if (act == GLFW_RELEASE && key == GLFW_KEY_SPACE) buttonRead.key_space = true;
}


// 鼠标按键事件处理
void UIctr::Mouse_button(int button, int act, int mods) {
    // 更新鼠标按键状态
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // 记录当前鼠标位置
    glfwGetCursorPos(window, &lastx, &lasty);
}


// 鼠标移动事件处理
void UIctr::Mouse_move(double xpos, double ypos) {
    // 无按键按下时不处理
    if (!button_left && !button_middle && !button_right) return;

    // 计算鼠标位移
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);  // 获取窗口尺寸

    // 检查 Shift 键状态
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // 确定鼠标操作类型（旋转、移动、缩放）
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // 移动相机（更新视角）
    mjv_moveCamera(mj_model, action, dx / height, dy / height, &scn, &cam);
}

// 鼠标滚动事件处理
void UIctr::Scroll(double xoffset, double yoffset) {
    // 模拟鼠标缩放（垂直方向）
    mjv_moveCamera(mj_model, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn, &cam);
}

// 关闭窗口并释放资源
void UIctr::Close() {
    // 释放 MuJoCo 资源
    mj_deleteData(mj_data);  // 释放数据
    mj_deleteModel(mj_model);  // 释放模型
    mjr_freeContext(&con);    // 释放渲染上下文
    mjv_freeScene(&scn);       // 释放场景

    glfwTerminate();  // 终止 GLFW 库
}
// 启用跟踪相机模式
void UIctr::enableTracking() {
    isTrack=true;
}
// 获取当前按键状态并重置
UIctr::ButtonState UIctr::getButtonState() {
    ButtonState tmp=buttonRead;
    // 重置按键状态（避免重复读取）
    buttonRead.key_w= false;
    buttonRead.key_a= false;
    buttonRead.key_s= false;
    buttonRead.key_d= false;
    buttonRead.key_h= false;
    buttonRead.key_j= false;
    buttonRead.key_space= false;
    return tmp;
}