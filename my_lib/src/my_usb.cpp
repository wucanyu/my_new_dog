#include "my_usb.h"
/*
    搜索串口设备
*/
std::vector<std::string> DOGUART::scanSerialDevices() {
    std::vector<std::string> deviceList;
    std::regex re("^ttyACM\\d+$");  // 匹配以 "ttyACM" 开头并跟着数字的设备文件名

    namespace bfs = boost::filesystem;
    for (bfs::directory_iterator it("/dev"); it != bfs::directory_iterator(); ++it) {
        std::string filename = it->path().filename().string();                                                                                     
        
        // 使用正则表达式检查设备名是否符合 "ttyACM" 格式
        if (std::regex_match(filename, re)) {
            deviceList.push_back(it->path().string());
        }
    }

    return deviceList;
}
/*
    搜索串口设备   
*/
void DOGUART::scan_usb()
{
    while (!serial_connected)
    {
        try
        {
            devices = scanSerialDevices();
            if (devices.empty()) 
            {
                std::cout << "未找到串口设备，正在重新扫描..." << std::endl;
                // 可以添加适当的延时，避免过于频繁地扫描，消耗过多资源
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            } 
            sp_ptr = new serial_port(iosev, devices[0]);
            // 设置波特率为115200
            sp_ptr->set_option(serial_port::baud_rate(115200));
            // 设置流控制为无（可以根据需要修改为硬件流控制或软件流控制）
            sp_ptr->set_option(serial_port::flow_control(serial_port::flow_control::none));
            // 设置奇偶校验为无（可以根据需要设置为奇校验或偶校验）
            sp_ptr->set_option(serial_port::parity(serial_port::parity::none));
            // 设置停止位为1位
            sp_ptr->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
            // 设置字符大小为8位
            sp_ptr->set_option(serial_port::character_size(8));
            serial_connected = true;
            std::cout << "串口已成功连接。" << std::endl;
        }   
        catch (const boost::system::system_error& e) 
        {
            std::cout << "无法打开串口: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

}
/*
    接受串口数据   
*/
void DOGUART::receive_usb()
{
        boost::system::error_code err;
        try 
        {
            bytes_read = sp_ptr->read_some(buffer(rxData, sizeof(rxData)), err);   
            if (err) //报错重新搜索串口
            {
                if (err == boost::asio::error::eof) {
                    std::cout << "串口连接已断开，尝试重新扫描串口设备..." << std::endl;
                    // 关闭当前串口
                    if (sp_ptr!= nullptr) {
                        sp_ptr->close();
                        delete sp_ptr;
                        sp_ptr = nullptr;
                    }
                    serial_connected = false;
                    // 进入重新扫描串口设备的循环
                    scan_usb();
                } else {
                    std::cout << "读取串口数据时出现其他错误: " << err.message() << std::endl;
                }
            } 
            else    
            {  //正常接收处理数据
            //    std::cout << "Read " << bytes_read << " bytes from serial port." << std::endl;
                // 进一步处理读取的数据，比如解析rxData
                const size_t float_size = sizeof(float);
                // 假设最多可以存储26个float数据
                size_t result_index = 0;

                if (rxData[0] == 0xAA && rxData[107] == 0xBB) {
                    result.resize(26);
                    /**************start****************/
                    // 从第1个字节开始，跳过起始标志0xAA
                    for (size_t i = 1; i < 105; i += float_size) {
                        if (result_index >= 26) {
                            break;
                        }
                        std::copy(&rxData[i], &rxData[i + float_size], reinterpret_cast<uint8_t*>(&result[result_index]));
                        result_index++;
                    }
                }
            }     
        } catch (const boost::system::system_error& e) {
            std::cout << "系统错误，读取串口数据时出现异常: " << e.what() << std::endl;     
        }
}
/*
    发送数据
*/
bool DOGUART::send_usb(const float* data, size_t data_size) {

    int index = 1;  // 用于跟踪 msg_temp 中的位置

    // 将输入的 float 数据依次复制到 msg_queue 中
    for (size_t i = 0; i < data_size; ++i) {
        memcpy(&msg_queue[index], &data[i], sizeof(float));
        index += sizeof(float);
    }

    // 串口写数据
    msg_queue[0] = 0xAA;
    msg_queue[33] = 0xBB;

    boost::system::error_code err;
    try 
    {
        sp_ptr->write_some(boost::asio::buffer(msg_queue, sizeof(msg_queue)), err);
        if (err) //报错重新搜索串口
        {
            std::cout << "串口连接已断开，尝试重新扫描串口设备..." << std::endl;
            // 关闭当前串口
            if (sp_ptr!= nullptr) {
                sp_ptr->close();
                delete sp_ptr;
                sp_ptr = nullptr;
            }
            serial_connected = false;
            // 进入重新扫描串口设备的循环
            scan_usb();
        } 
        else    
        { 
        }     
    } catch (const boost::system::system_error& e) {
        std::cout << "系统错误，发送串口数据时出现异常: " << e.what() << std::endl;     
    }

    return true;
}


 