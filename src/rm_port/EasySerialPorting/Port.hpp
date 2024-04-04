/*** 
 * @Author: gaoyuan
 * @Date: 2023-04-02 11:48:56
 * @LastEditors: gaoyuan
 * @LastEditTime: 2023-04-18 19:14:23
 */
#pragma once
#pragma once
#include <string>
#include <vector>
#include <boost/asio.hpp>
#include <sstream>
#include <iomanip>


namespace Easy
{
    class Port
    {
    public:
        /// @brief 构造一个串口对象并绑定设备地址
        /// @param path 设备地址
        explicit Port(std::string path, int baud_rate);
        /// 析构时关闭串口对象
        virtual ~Port();

    protected:
        /// 设备路径
        const std::string Path;

        /// 上下文对象
        boost::asio::io_context Context;
        /// 串口设备对象
        boost::asio::serial_port Device;

    public:
        /**
         * @brief 将字节写入到串口
         * @param bytes 需要写入的字节，全部都会被写入到设备中
         * @details 该函数将自动添加包头和包尾。
         */
        void Write(const std::vector<unsigned char>& bytes);

        /**
         * @brief 从设备中读取数据
         * @param bytes 用于存储读取的字节的容器
         * @details 将自动去除包头包尾
         */
        bool Read(std::vector<unsigned char>& bytes);

        /**
         * @brief 打开串口
         */
        void Open();

        /**
         * @brief 关闭串口
         */
        void Close();
        std::string BytesToHexStr(const std::vector<unsigned char> &byteArray);
    };
}
