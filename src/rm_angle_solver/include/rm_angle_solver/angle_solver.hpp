/*** 
 * @Author: gaoyuan
 * @Date: 2023-05-31 03:03:29
 * @LastEditors: gaoyuan
 * @LastEditTime: 2023-06-20 18:14:54
 */
#pragma once

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/angle.hpp"
#include "auto_aim_interfaces/msg/recieve_data.hpp"
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Eigen>

#include <mutex>

namespace rm_angle_solver
{
//用于存储目标装甲板的信息
struct TargetArmorPosition
{
    float x;           //装甲板在世界坐标系下的x
    float y;           //装甲板在世界坐标系下的y
    float z;           //装甲板在世界坐标系下的z
    float yaw;         //装甲板坐标系相对于世界坐标系的yaw角
};

enum ARMOR_NUM
{
    ARMOR_NUM_BALANCE = 2,
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
};

class RMAngleSolver : public rclcpp::Node
{
public:
    explicit RMAngleSolver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~RMAngleSolver() = default;

private:
    void angleSolverCallBackAsync();
    
    void angleSolverCallBack(const auto_aim_interfaces::msg::Target::SharedPtr msg);

    void robotStatusCallBack(const auto_aim_interfaces::msg::RecieveData::SharedPtr msg);

    void targetCallBack(const auto_aim_interfaces::msg::Target::SharedPtr msg);
    
    /**
     * @brief 根据目标的运动状态和自己云台的运动状态选择打击装甲板
     * @param msg 目标的运动状态
     * @param predict_time 预测时间
     * @param robot_status pitch,yaw角度
     * @return Eigen::Matrix<double, 6, 1> 选择装甲板在一定预测时间后的三维坐标和预测时间后1ms的三维坐标
     */
    Eigen::Matrix<double, 6, 1> choose_target(const auto_aim_interfaces::msg::Target& msg, const Eigen::Vector2d& robot_status);

    /**
     * @brief 解算弹道
     * @param pose 目标装甲板的三维坐标
     * @return Eigen::Vector2d pitch，yaw轴角度(角度制)
     */
    Eigen::Vector2d solve_angles(const Eigen::Vector3d& pose);

    /**
     * @brief 解算弹道
     * @param pose 目标装甲板的三维坐标
     * @return Eigen::Vector2d pitch轴角度(角度制)
     */
    double solve_pitch_angle(const Eigen::Vector3d& pose);

    /**
     * @brief 解算弹道
     * @param pose 目标装甲板的三维坐标
     * @return Eigen::Vector2d yaw轴角度(角度制)
     */
    double solve_yaw_angle(const Eigen::Vector3d& pose);
    void update_params();

    auto_aim_interfaces::msg::Angle angle_msg;
    visualization_msgs::msg::Marker aiming_point_;

    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::RecieveData>::SharedPtr robot_status_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Angle>::SharedPtr target_angles_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double x_offset;
    double y_offset;
    double z_offset;
    double yaw_offset;
    double pitch_offset;
    int max_iter;
    double stop_error;
    int R_K_iter;
    double g;
    double k;
    double delay_time;
    bool fix_frequency;
    double frequency;
    bool send_pitch_speed;

    std::mutex data_lock;
    double shoot_speed;
    double robot_pitch;
    double robot_pitch_speed;
    double robot_yaw;
    double robot_yaw_speed;
    auto_aim_interfaces::msg::Target target_msg;

    TargetArmorPosition target_armor_position[4];
};
}

