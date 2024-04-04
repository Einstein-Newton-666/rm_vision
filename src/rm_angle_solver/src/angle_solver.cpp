/*** 
 * @Author: gaoyuan
 * @Date: 2023-05-31 03:03:42
 * @LastEditors: gaoyuan
 * @LastEditTime: 2023-06-19 20:24:45
 */
#include <cmath>
#include "rm_angle_solver/angle_solver.hpp"

namespace rm_angle_solver
{
RMAngleSolver::RMAngleSolver(const rclcpp::NodeOptions & options)
: Node("rm_angle_solver", options)
{   
    RCLCPP_INFO(this->get_logger(), "Starting AngleSolverNode!");


    //设置默认值
    robot_pitch = 0;
    robot_pitch_speed = 0;
    robot_yaw = 0;
    robot_yaw_speed = 0;
    shoot_speed = declare_parameter<double>("default_shoot_speed", 28.);

    x_offset = declare_parameter<double>("x_offset", 0.);
    y_offset = declare_parameter<double>("y_offset", 0.);
    z_offset = declare_parameter<double>("z_offset", 0.);
    yaw_offset = declare_parameter<double>("yaw_offset", 0.);
    pitch_offset = declare_parameter<double>("pitch_offset", 0.);
    max_iter = declare_parameter<int>("max_iter", 10.);
    stop_error = declare_parameter<double>("stop_error", 0.001);
    R_K_iter = declare_parameter<int>("R_K_iter", 60.);
    g = declare_parameter<double>("g", 9.781);
    k = declare_parameter<double>("k", 0.1903);
    delay_time = declare_parameter<double>("delay_time", 0.03);
    fix_frequency = this->declare_parameter<bool>("fix_frequency", true);
    frequency = this->declare_parameter<int>("frequency", 200);
    send_pitch_speed = this->declare_parameter<bool>("send_pitch_speed", false);

    aiming_point_.header.frame_id = "odom";
    aiming_point_.ns = "aiming_point";
    aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
    aiming_point_.action = visualization_msgs::msg::Marker::ADD;
    aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
    aiming_point_.color.r = 1.0;
    aiming_point_.color.g = 1.0;
    aiming_point_.color.b = 1.0;
    aiming_point_.color.a = 1.0;
    aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/aiming_point", 10
    );    

    robot_status_sub_  = this->create_subscription<auto_aim_interfaces::msg::RecieveData>(
        "/recieve_pack", rclcpp::SensorDataQoS(),
        std::bind(&RMAngleSolver::robotStatusCallBack, this, std::placeholders::_1));
    target_angles_pub_  = this->create_publisher<auto_aim_interfaces::msg::Angle>(
        "/target_angles", rclcpp::SensorDataQoS());
    if(fix_frequency){
        target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
            "/tracker/target", rclcpp::SensorDataQoS(),
            std::bind(&RMAngleSolver::targetCallBack, this, std::placeholders::_1)
        );
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000./frequency)), 
            std::bind(&RMAngleSolver::angleSolverCallBackAsync, this)
        );
    }
    else{
        target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
            "/tracker/target", rclcpp::SensorDataQoS(),
            std::bind(&RMAngleSolver::angleSolverCallBack, this, std::placeholders::_1)
        );
    }
}

void RMAngleSolver::targetCallBack(const auto_aim_interfaces::msg::Target::SharedPtr msg){
    // data_lock.lock();
    target_msg = *msg;
    // data_lock.unlock();
}

void RMAngleSolver::robotStatusCallBack(const auto_aim_interfaces::msg::RecieveData::SharedPtr msg){
    shoot_speed = msg->shoot_speed;
    robot_pitch = msg->pitch/180*M_PI;
    robot_yaw = msg->yaw/180*M_PI;
}

void RMAngleSolver::angleSolverCallBackAsync(){
    update_params();
    if(target_msg.tracking){
        auto target_pose = choose_target(target_msg, Eigen::Vector2d{robot_pitch,robot_yaw});
        auto angle = solve_angles(Eigen::Vector3d{target_pose[0],target_pose[1],target_pose[2]});
        Eigen::Vector2d delta_angle;
        if(send_pitch_speed){
            delta_angle = solve_angles(Eigen::Vector3d{target_pose[3],target_pose[4],target_pose[5]});
        }
        else{
            auto delta_yaw = solve_yaw_angle(Eigen::Vector3d{target_pose[3],target_pose[4],target_pose[5]});
            delta_angle = {angle[0], delta_yaw};
        }
        auto angle_speed = (delta_angle - angle) / 0.001;

        angle_msg.header.stamp = target_msg.header.stamp;
        angle_msg.pitch = angle[0];
        angle_msg.yaw =-angle[1];
        angle_msg.pitch_speed = angle_speed[0];
        angle_msg.yaw_speed = angle_speed[1];
        target_angles_pub_->publish(angle_msg);
        
        //可视化选择的装甲板
        aiming_point_.header.stamp = this->now();
        aiming_point_.pose.position.x = target_pose[0];
        aiming_point_.pose.position.y = target_pose[1];
        aiming_point_.pose.position.z = target_pose[2];
        marker_pub_->publish(aiming_point_);
        
        //更新状态
        robot_pitch += robot_pitch_speed * 1. / frequency;
        robot_yaw += robot_yaw_speed * 1. / frequency;
        target_msg.position.x += target_msg.velocity.x * 1. / frequency;
        target_msg.position.y += target_msg.velocity.y * 1. / frequency;
        target_msg.position.z += target_msg.velocity.z * 1. / frequency;
        target_msg.yaw += target_msg.v_yaw  * 1. / frequency;
    }
    else{
        angle_msg.header.stamp = target_msg.header.stamp;
        angle_msg.pitch = 0;
        angle_msg.yaw = 0;
        angle_msg.pitch_speed = 0;
        angle_msg.yaw_speed = 0;
        target_angles_pub_->publish(angle_msg);
    }
}

void RMAngleSolver::angleSolverCallBack(const auto_aim_interfaces::msg::Target::SharedPtr msg){
    update_params();
    if(msg->tracking){
        auto target_pose = choose_target(*msg, Eigen::Vector2d{robot_pitch, robot_yaw});
        auto angle = solve_angles(Eigen::Vector3d{target_pose[0],target_pose[1],target_pose[2]});
        Eigen::Vector2d delta_angle;
        if(send_pitch_speed){
            delta_angle = solve_angles(Eigen::Vector3d{target_pose[3],target_pose[4],target_pose[5]});
        }
        else{
            auto delta_yaw = solve_yaw_angle(Eigen::Vector3d{target_pose[3],target_pose[4],target_pose[5]});
            delta_angle = {angle[0], delta_yaw};
        }
        auto angle_speed = (delta_angle - angle) / 0.001;

        angle_msg.header.stamp = msg->header.stamp;
        angle_msg.pitch = angle[0];
        angle_msg.yaw =-angle[1];
        angle_msg.pitch_speed = angle_speed[0];
        angle_msg.yaw_speed = angle_speed[1];
        target_angles_pub_->publish(angle_msg);
        
        //可视化选择的装甲板
        aiming_point_.header.stamp = this->now();
        aiming_point_.pose.position.x = target_pose[0];
        aiming_point_.pose.position.y = target_pose[1];
        aiming_point_.pose.position.z = target_pose[2];
        marker_pub_->publish(aiming_point_);
        
    }
    else{
        angle_msg.header.stamp = msg->header.stamp;
        angle_msg.pitch = 0;
        angle_msg.yaw = 0;
        angle_msg.pitch_speed = 0;
        angle_msg.yaw_speed = 0;
        target_angles_pub_->publish(angle_msg);
    }
}


Eigen::Matrix<double, 6, 1> RMAngleSolver::choose_target(const auto_aim_interfaces::msg::Target &msg, 
    const Eigen::Vector2d &robot_status){
    //计算四块装甲板的位置
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
	int use_1 = 1;
	int i = 0;
    int idx = 0; // 选择的装甲板
    //平衡步兵
    if (msg.armors_num == ARMOR_NUM_BALANCE) {
        for (i = 0; i<2; i++) {
            float tmp_yaw = msg.yaw + i * M_PI;
            float r = msg.radius_1;
            target_armor_position[i].x = msg.position.x - r*cos(tmp_yaw);
            target_armor_position[i].y = msg.position.y - r*sin(tmp_yaw);
            target_armor_position[i].z = msg.position.z;
            target_armor_position[i].yaw = tmp_yaw;
        }
        float yaw_diff_min = fabsf(robot_status[1] - target_armor_position[0].yaw);
        //因为是平衡步兵 只需判断两块装甲板即可
        float temp_yaw_diff = fabsf(robot_status[1] - target_armor_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }
    } else if (msg.armors_num == ARMOR_NUM_OUTPOST) {  //前哨站
        for (i = 0; i<3; i++) {
            float tmp_yaw = msg.yaw + i * 2.0 * M_PI/3.0;  // 2/3PI
            float r =  (msg.radius_1 + msg.radius_2)/2;   //理论上r1=r2 这里取个平均值
            target_armor_position[i].x = msg.position.x - r*cos(tmp_yaw);
            target_armor_position[i].y = msg.position.y - r*sin(tmp_yaw);
            target_armor_position[i].z = msg.position.z;
            target_armor_position[i].yaw = tmp_yaw;
        }
        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用
        //计算枪管到目标装甲板yaw最小的那个装甲板
        float yaw_diff_min = fabsf(robot_status[1] - target_armor_position[0].yaw);
        for (i = 1; i<3; i++) {
            float temp_yaw_diff = fabsf(robot_status[1] - target_armor_position[i].yaw);
            if (temp_yaw_diff < yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }
    } else {
        //正常装甲板
        for (i = 0; i<4; i++) {
            float tmp_yaw = msg.yaw + i * M_PI/2.0;
            float r = use_1 ? msg.radius_1 : msg.radius_2;
            target_armor_position[i].x = msg.position.x - r*cos(tmp_yaw);
            target_armor_position[i].y = msg.position.y - r*sin(tmp_yaw);
            target_armor_position[i].z = use_1 ? msg.position.z : msg.position.z + msg.dz;
            target_armor_position[i].yaw = tmp_yaw;
            use_1 = !use_1;
        }

        //2种常见决策方案：
        //1.计算枪管到目标装甲板yaw最小的那个装甲板
        //2.计算距离最近的装甲板


      
        //计算距离最近的装甲板
        	// float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
        	// int idx = 0;
        	// for (i = 1; i<4; i++)
        	// {
        	// 	float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
        	// 	if (temp_dis_diff < dis_diff_min)
        	// 	{
        	// 		dis_diff_min = temp_dis_diff;
        	// 		idx = i;
        	// 	}
        	// }
          //计算枪管到目标装甲板yaw最小的那个装甲板
        float yaw_diff_min = fabsf(robot_status[1] - target_armor_position[0].yaw);
        for (i = 1; i<4; i++) {
            float temp_yaw_diff = fabsf(robot_status[1] - target_armor_position[i].yaw);
            if (temp_yaw_diff < yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }

    }
    
    //用匀速运动模型估算飞行时间
    double shoot_delay = sqrt(target_armor_position[idx].x * target_armor_position[idx].x +
                            target_armor_position[idx].y * target_armor_position[idx].y +
                            target_armor_position[idx].z * target_armor_position[idx].z) / shoot_speed;
    //预测时间等于飞行时间加固定延迟
    double aim_x = target_armor_position[idx].x + msg.velocity.x * (shoot_delay + delay_time);
    double aim_y = target_armor_position[idx].y + msg.velocity.y * (shoot_delay + delay_time);
    double aim_z = target_armor_position[idx].z + msg.velocity.z * (shoot_delay + delay_time);
    double pre_aim_x = target_armor_position[idx].x + msg.velocity.x * (shoot_delay + delay_time + 0.001);
    double pre_aim_y = target_armor_position[idx].y + msg.velocity.y * (shoot_delay + delay_time + 0.001);
    double pre_aim_z = target_armor_position[idx].z + msg.velocity.z * (shoot_delay + delay_time + 0.001);
    Eigen::Matrix<double, 6, 1> pose_data;
    pose_data << aim_x, aim_y, aim_z, pre_aim_x, pre_aim_y, pre_aim_z;
    return pose_data;
}

Eigen::Vector2d RMAngleSolver::solve_angles(const Eigen::Vector3d& pose){
    auto yaw = solve_yaw_angle(pose)+yaw_offset;
    auto pitch = solve_pitch_angle(pose)+pitch_offset;
    return Eigen::Vector2d{pitch, yaw};
}

double RMAngleSolver::solve_pitch_angle(const Eigen::Vector3d& pose){
    //TODO:根据陀螺仪安装位置调整距离求解方式
    //使用最新的弹速而不是时间戳对其的弹速                         
    auto new_shoot_speed = double(shoot_speed);
    auto high = pose[2];
    auto OffsetedHigh = high;
    auto distance_x = sqrt(pose.squaredNorm() - high * high);
    auto pitch = atan(high / distance_x) * 180 / M_PI;
    auto pitch_new = pitch;
    //开始使用龙格库塔法求解弹道补偿
    for (int i = 0; i < max_iter; i++)
    {
        //TODO:可以考虑将迭代起点改为世界坐标系下的枪口位置
        //初始化
        auto x = x_offset;
        auto y = y_offset;
        auto p = tan(pitch_new / 180 * M_PI);
        auto v = new_shoot_speed;
        auto u = v / sqrt(1 + pow(p,2));
        auto delta_x = distance_x / R_K_iter;
        for (int j = 0; j < R_K_iter; j++)
        {
            auto k1_u = -k * u * sqrt(1 + pow(p, 2));
            auto k1_p = -g / pow(u, 2);
            auto k1_u_sum = u + k1_u * (delta_x / 2);
            auto k1_p_sum = p + k1_p * (delta_x / 2);

            auto k2_u = -k * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
            auto k2_p = -g / pow(k1_u_sum, 2);
            auto k2_u_sum = u + k2_u * (delta_x / 2);
            auto k2_p_sum = p + k2_p * (delta_x / 2);

            auto k3_u = -k * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
            auto k3_p = -g / pow(k2_u_sum, 2);
            auto k3_u_sum = u + k3_u * (delta_x / 2);
            auto k3_p_sum = p + k3_p * (delta_x / 2);

            auto k4_u = -k * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
            auto k4_p = -g / pow(k3_u_sum, 2);

            u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
            p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

            x+=delta_x;
            y+=p * delta_x;
        }
        //评估迭代结果,若小于迭代精度需求则停止迭代
        auto error = high - y;
        if (abs(error) <= stop_error)
        {
            break;
        }
        else
        {
            OffsetedHigh+=error;
            pitch_new = atan(OffsetedHigh / distance_x) * 180 / M_PI;
        }
    }
    return pitch_new;
}

double RMAngleSolver::solve_yaw_angle(const Eigen::Vector3d& pose){
    double yaw = atan2(pose[1], pose[0]) * 180 / M_PI;
    return yaw;
}

void RMAngleSolver::update_params(){
    shoot_speed =get_parameter("default_shoot_speed").as_double();
    x_offset =get_parameter("x_offset").as_double();
    y_offset = get_parameter("y_offset").as_double();
    z_offset = get_parameter("z_offset").as_double();
    yaw_offset = get_parameter("yaw_offset").as_double();
    pitch_offset = get_parameter("pitch_offset").as_double();
    max_iter = get_parameter("max_iter").as_int();
    stop_error = get_parameter("stop_error").as_double();
    R_K_iter = get_parameter("R_K_iter").as_int();
    g = get_parameter("g").as_double();
    k =get_parameter("k").as_double();
    delay_time =get_parameter("delay_time").as_double();
    fix_frequency = get_parameter("fix_frequency").as_bool();
    frequency =get_parameter("frequency").as_int();
    send_pitch_speed = get_parameter("send_pitch_speed").as_bool();  
}


}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_angle_solver::RMAngleSolver)
