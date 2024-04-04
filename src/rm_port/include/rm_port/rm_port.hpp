/*** 
 * @Author: gaoyuan
 * @Date: 2023-03-06 00:37:03
 * @LastEditors: gaoyuan
 * @LastEditTime: 2023-06-22 07:24:08
 */
#pragma once
#include "Send.pb.h"
#include "Recieve.pb.h"
#include "Port.hpp"

#include <vector>
#include <mutex>
#include <memory>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include "auto_aim_interfaces/msg/recieve_data.hpp"
#include "auto_aim_interfaces/msg/angle.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_ros/buffer.h"

namespace auto_aim_port
{

class rm_port: public rclcpp::Node{

/////////////////////
//在下面定义自己的串口//
/////////////////////装甲板
public:
    rm_port(const rclcpp::NodeOptions & options);
    ~rm_port();

private:

    void pub_recieve_pack();

    void sendCallback(const auto_aim_interfaces::msg::Angle::SharedPtr msg);

    void resetTracker();
    
    rclcpp::Publisher<auto_aim_interfaces::msg::RecieveData>::SharedPtr recieve_pub_;

    rclcpp::Subscription<auto_aim_interfaces::msg::Angle>::SharedPtr send_sub_;
    
      // Service client to reset tracker
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

    // Broadcast tf from odom to gimbal_link
    double timestamp_offset_ = 0;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;

    //如果长时间未发送数据认为程序出了问题，需要重新启动
    bool active;
    double check_frequency;
    rclcpp::TimerBase::SharedPtr status_checker;

////////////////
//下面不需要修改//
////////////////

private:
    /**
     * 给发送帧赋值(使用std::move)
     * @return
     */
    inline void SetSendFrame(host_to_device::Frame& frame){
        SendFrame = std::move(frame);
    }
    /**
     * 获得接收帧的拷贝
     * 线程安全上的🔓qwq
     * @return
     */
    device_to_host::Frame GetReceiveFrame(){
        auto res = ReceiveFrame;
        
        return res;
    }

    /**
     * @brief 在串口断开的情况下重新打开串口
     */
    void ReOpenPort(){
        RCLCPP_WARN(this->get_logger(),"Attempting to reopen the port");
        try{
            Port->Close();
            Port->Open();
            RCLCPP_INFO(this->get_logger(),"Successfully open the port");
        } catch (const std::exception &e){
            RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", e.what());
            if(rclcpp::ok()){
                rclcpp::sleep_for(std::chrono::seconds(1));
                ReOpenPort();
            } else {
                exit(0);
            }
        }
    };

    bool Receive(){
        try{
            if(Port == nullptr){
                return false;
            }
            Port->Read(ReceiveBuffer);
            if(ReceiveBuffer.size()>65535){
                RCLCPP_ERROR(this->get_logger(),"The port size is larger than 65535");
            }
            ReceiveFrame.ParseFromArray(ReceiveBuffer.data(),ReceiveBuffer.size());
            return true;
        }
        catch(const std::exception &e){
            RCLCPP_ERROR(get_logger(), "Error while Recieveing data: %s", e.what());
            ReOpenPort();
            return false;
        }
    };

    void Send(){
        try{
            if(Port == nullptr){
                return;
            }
            SendBuffer.clear();
            SendBuffer.resize(SendFrame.ByteSizeLong());
            // std::cout<<SendBuffer.size()<<std::endl;
            SendFrame.SerializeToArray(SendBuffer.data(), SendBuffer.size());
            Port->Write(SendBuffer);
        }
        catch(const std::exception &e){
            RCLCPP_ERROR(get_logger(), "Error while sending data: %s", e.what());
            ReOpenPort();
        }
    };

    host_to_device::Frame SendFrame;
    device_to_host::Frame ReceiveFrame;
    std::vector<unsigned char>SendBuffer;
    std::vector<unsigned char>ReceiveBuffer;
    std::shared_ptr<Easy::Port>Port = nullptr;
    std::string SerialPort_PortPath;
    int baud_rate;
    bool limit_recieve_hz;
    int recieve_hz;
    std::thread receive_thread_;
};

}