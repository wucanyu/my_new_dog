#include "dog.h"

int main(int argc, char **argv) {
    // create controller
    std::unique_ptr<HardwareDog> dog = std::make_unique<HardwareDog>();
    // 控制执行标志
    std::atomic<bool> control_execute{};
    control_execute.store(true, std::memory_order_release);
    // 延时2秒
    sleep(1);  
    // Thread 1: compute desired ground forces
    std::cout << "Enter thread 1: compute desired ground forces" << std::endl;
    std::thread compute_foot_forces_grf_thread([&]() {
        // 使用std::chrono::high_resolution_clock来获取时间
        auto start = std::chrono::high_resolution_clock::now();
        auto prev = start;
        // prepare variables to monitor time and control the while loop
        while (control_execute.load(std::memory_order_acquire)) {

            auto now = std::chrono::high_resolution_clock::now();
            auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - prev);
            prev = now;  

            // bool running = dog->update_foot_forces_grf(dt.count());
            bool running = dog->update_foot_forces_grf(dt.count());
            if (!running) {
                std::cout << "Thread 1 loop is terminated because of errors." << std::endl;
                std::terminate();
                break;
            }
        }
    });

    // Thread 2: update robot states, compute desired swing legs forces, compute desired joint torques, and send commands
    std::cout << "Enter thread 2: update robot states, compute desired swing legs forces, compute desired joint torques, and send commands"
      << std::endl;
    std::thread main_thread([&]() {
        auto start = std::chrono::high_resolution_clock::now();
        auto prev = start;
        // prepare variables to monitor time and control the while loop
        while (control_execute.load(std::memory_order_acquire)) {
            // auto t3 = std::chrono::high_resolution_clock::now();

            // get t and dt
            auto now = std::chrono::high_resolution_clock::now();
            auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - prev);
            prev = now;
            auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start);

            // compute desired ground forces
            bool main_update_running = dog->main_update(elapsed.count(), dt.count());
            bool send_cmd_running = true;//dog->dog_usb.send_usb();
            // dog->send_data();
            // auto t4 = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double, std::milli> ms_double = t4 - t3;
            // std::cout << "Thread 2 is updated in " << dt.toSec()<< "s" << std::endl;

            if (!main_update_running || !send_cmd_running) {
                std::terminate();
                break;
            }
        }
    });

    compute_foot_forces_grf_thread.join();
    main_thread.join();

    return 0;
}
