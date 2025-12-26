#ifndef CPP_MYUSB_H
#define CPP_MYUSB_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
 
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <regex>
#include <string>
#include "mid_data.h"

using namespace std;        // IO 服务，用于处理异步操作
using namespace boost::asio;
using namespace boost::placeholders;

class DOGUART{
public:
    /*
        搜索串口设备
    */
    std::vector<std::string> scanSerialDevices();
    void scan_usb();
    void receive_usb();
    bool send_usb(const float* data, size_t data_size); 
    // 访问器方法，获取数组元素
    // 若索引无效，返回 -1 表示错误
    float getArrayElement(int index) const {
        if (index >= 0 && index < result.size()) {
            return result[index];
        }
        // 索引无效，返回 -1 作为错误标识
        return -1; 
    }

    uint8_t get_rxData(int index) const {
        return rxData[index]; 
    }

    // 导入外部数据的函数
    void importData(const float inputData[8]) {
        for (int i = 0; i < 8; ++i) {
            txData[i] = inputData[i];
        }
    }

private:
    std::vector<std::string> devices;
    io_service iosev;
    // 创建serial_port对象指针
    serial_port* sp_ptr = nullptr; 
    boost::system::error_code err;  
    bool serial_connected = false;
    //接受数据的长度
    size_t bytes_read;
    // 串口固定消息队列
    uint8_t msg_queue[34];
    // 串口固定接受缓冲
    uint8_t rxData[108];
    // 串口固定发送缓冲
    float txData[8];
    //中间数据
    std::vector<float> result;

};

#endif 
