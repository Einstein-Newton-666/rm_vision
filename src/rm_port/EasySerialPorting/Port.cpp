/*
 * @Author: 2476240435 2476240435@qq.com
 * @Date: 2023-10-13 21:09:41
 * @LastEditors: 2476240435 2476240435@qq.com
 * @LastEditTime: 2023-11-29 01:31:38
 * @FilePath: /rm_vision/src/rm_port/EasySerialPorting/Port.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*** 
 * @Author: gaoyuan
 * @Date: 2023-05-30 00:23:04
 * @LastEditors: gaoyuan
 * @LastEditTime: 2023-06-16 12:05:17
 */
#include "Port.hpp"
#include "CRCTool.hpp"
#include <iostream>

namespace Easy
{
    /// 构造函数
    Port::Port(std::string path, int baud_rate) : Path(std::move(path)), Context(), Device(Context)
    {
        using namespace boost::asio;

        // 开启串口
        if (!Device.is_open())
        {
            Device.open(Path);
        }

        // 配置串口设备
        Device.set_option(serial_port_base::baud_rate(baud_rate));
        Device.set_option(serial_port_base::character_size(8));
        Device.set_option(serial_port_base::flow_control(serial_port::flow_control::none));
        Device.set_option(serial_port_base::parity(serial_port::parity::none));
        Device.set_option(serial_port_base::stop_bits(serial_port::stop_bits::one));
    }

    /// 析构时关闭对象
    Port::~Port()
    {
        if (Device.is_open())
        {
            Device.close();
        }
    }

    /// 将内容写入到设备
    void Port::Write(const std::vector<unsigned char> &content)
    {
        std::vector<unsigned char> head, tail;
        head.resize(2);
        head[0] = 0xBE;
        *(unsigned char*)&head[1] = content.size();
        tail.resize(2);
        tail[0] = Utilities::CRCTool::GetCRC8CheckSum(content.data(), content.size());
        tail[1] = 0xED;

        boost::asio::write(Device, boost::asio::buffer(head.data(), head.size()));
        boost::asio::write(Device, boost::asio::buffer(content.data(), content.size()));
        boost::asio::write(Device, boost::asio::buffer(tail.data(), tail.size()));
    }

    /// 从设备中读取内容
    bool Port::Read(std::vector<unsigned char> &bytes)
    {
        
        // std::string hexstring = BytesToHexStr(bytes);
        // std::cout<<"Hex String:"<<hexstring<<std::endl;
        unsigned char head = 0;

        while (head != 0xBE)
            boost::asio::read(Device, boost::asio::buffer(&head, 1));

        boost::asio::read(Device, boost::asio::buffer(&head, 1));

        bytes.resize(head);

        //std::cout << "Length:" << (int)head << std::endl;

        boost::asio::read(Device, boost::asio::buffer(bytes.data(), bytes.size()));

        std::vector<unsigned char> information;
        information.resize(2);
        boost::asio::read(Device, boost::asio::buffer(information.data(), information.size()));
        if (information[1] != 0xED)
        {
            std::cout << "Wrong ED Flag:" << information[1] << std::endl;
            return false;
        }

        auto check = Utilities::CRCTool::GetCRC8CheckSum(bytes.data(), bytes.size());
        // std::cout << "Cal CRC:" << (int)check << std::endl;
        if (check != information[0])
        {
            std::cout << "Wrong CRC:" << (int)information[0] << std::endl;
            return false;
        }


        // std::cout << "CRC passed." << std::endl;

        return true;
    }

    void Port::Open()
    {
        if (!Device.is_open())
        {
            Device.open(Path);
        }
    }

    void Port::Close(){
        if (Device.is_open())
        {
            Device.close();
        }
    }
    std::string Port::BytesToHexStr(const std::vector<unsigned char>&byteArray)
{
    std::stringstream ss;
    ss<<std::hex<<std::setfill(' ');
    for(unsigned char byte:byteArray){
        ss<<std::setw(4)<<static_cast<int>(byte);
    }
    return ss.str();
}

}