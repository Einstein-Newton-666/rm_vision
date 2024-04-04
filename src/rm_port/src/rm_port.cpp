/*** 
 * @Author: gaoyuan
 * @Date: 2023-03-05 21:40:36
 * @LastEditors: gaoyuan
 * @LastEditTime: 2023-06-22 18:25:05
 */
#include "rm_port/rm_port.hpp"
namespace auto_aim_port{


rm_port::rm_port(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()):
Node("rm_port_node",options)
{   
    RCLCPP_INFO(this->get_logger(),"Start SerialPort");

    SerialPort_PortPath = declare_parameter<std::string>("SerialPort_PortPath", "/dev/ttyUSB0");
    baud_rate = declare_parameter<int>("baud_rate", 115200);
    timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);


    recieve_pub_ = this->create_publisher<auto_aim_interfaces::msg::RecieveData>(
        "/recieve_pack", rclcpp::SensorDataQoS()
    );

    send_sub_ = this->create_subscription<auto_aim_interfaces::msg::Angle>(
        "/target_angles", rclcpp::SensorDataQoS(),
        std::bind(&rm_port::sendCallback, this, std::placeholders::_1)
    );

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);

    // Tracker reset service client
    reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

    try{
        Port = std::make_shared<Easy::Port>(SerialPort_PortPath, baud_rate);
        RCLCPP_INFO(this->get_logger(),"open the port");
    }catch(const std::exception& e){
        RCLCPP_ERROR_STREAM(this->get_logger(),e.what() + SerialPort_PortPath);
        throw(e);
    }

    receive_thread_ = std::thread{[this]() -> void {
        while(rclcpp::ok()){
            if(Receive()){
                pub_recieve_pack();
            }
        }
    }};
    RCLCPP_INFO(this->get_logger(),"start recieve pack");

    active = true;
    // check_frequency = declare_parameter<double>("check_frequency", 0.5);
    // status_checker = this->create_wall_timer(
    //     std::chrono::milliseconds(int(1000/check_frequency)), 
    //     [this](){
    //         if(active){
    //             active = false;
    //         } else {
    //             RCLCPP_ERROR(this->get_logger(), "Long time no action, shutdown!");
    //             rclcpp::shutdown();
    //         }
    //     }
    // );
    RCLCPP_INFO(this->get_logger(),"start send pack");
}

rm_port::~rm_port(){
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "Stop rm_port");
}


void rm_port::pub_recieve_pack(){
    if(rclcpp::ok()){
        auto recieve_pack = this->GetReceiveFrame();
        timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
        rclcpp::Time now_time = rclcpp::Clock().now() + rclcpp::Duration::from_seconds(timestamp_offset_);

        auto_aim_interfaces::msg::RecieveData pub_pack;
        pub_pack.header.stamp = now_time;
        pub_pack.pitch = recieve_pack.current_pitch_();
        pub_pack.yaw = recieve_pack.current_yaw_();
    
        pub_pack.shoot_speed = recieve_pack.bullet_speed_();
        if(recieve_pack.current_color_() > 100){
            //蓝色
            pub_pack.current_color = 1;
        } else{
            //红色
            pub_pack.current_color = 0;
        }
        pub_pack.enemies_blood_0 = recieve_pack.enemies_blood_0();
        pub_pack.enemies_blood_1 = recieve_pack.enemies_blood_1();
        pub_pack.enemies_blood_2 = recieve_pack.enemies_blood_2();
        pub_pack.enemies_blood_3 = recieve_pack.enemies_blood_3();
        pub_pack.enemies_blood_4 = recieve_pack.enemies_blood_4();
        pub_pack.enemies_blood_5 = recieve_pack.enemies_blood_5();
        pub_pack.enemies_outpost = recieve_pack.enemies_outpost();
        pub_pack.attack_engineer = recieve_pack.ifattackengineer();
        recieve_pub_->publish(pub_pack);

        if(recieve_pack.reset_tracker()){
            resetTracker();
        }

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "gimbal_link";
        tf2::Quaternion q;
        double roll = recieve_pack.current_roll_()*M_PI/180;
        double pitch = recieve_pack.current_pitch_()*M_PI/180;
        double yaw = recieve_pack.current_yaw_()*M_PI/180;
        q.setRPY(roll, -pitch, -yaw);
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(t);


        RCLCPP_DEBUG(this->get_logger(), "publish recieve data");
        
    }

}

void rm_port::sendCallback(const auto_aim_interfaces::msg::Angle::SharedPtr msg){
    active = true;
    
    host_to_device::Frame send_pack;
    send_pack.set_target_pitch_(msg->pitch);
    send_pack.set_target_yaw_(msg->yaw);
    send_pack.set_pitch_speed(msg->pitch_speed);
    send_pack.set_yaw_speed(msg->yaw_speed);
    this->SetSendFrame(send_pack);
    Send();

    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);
}

void rm_port::resetTracker()
{
    if (!reset_tracker_client_->service_is_ready()) {
        RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    reset_tracker_client_->async_send_request(request);
    RCLCPP_INFO(get_logger(), "Reset tracker!");
}


}//namespace auto_aim_port
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(auto_aim_port::rm_port)
