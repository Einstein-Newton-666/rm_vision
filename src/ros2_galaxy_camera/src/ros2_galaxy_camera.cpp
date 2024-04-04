/*** 
 * @Author: gaoyuan
 * @Date: 2023-04-07 10:48:12
 * @LastEditors: gaoyuan
 * @LastEditTime: 2023-07-18 18:39:21
 */
#include <ros2_galaxy_camera/ros2_galaxy_camera.hpp>

using namespace ros2_galaxy_camera;

galaxy_camera_node::galaxy_camera_node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
: Node("galaxy_camera", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting galaxy_camera_node!");
    GXInitLib();
    auto status = GXInitLib();
    if (status!= GX_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(),"GX lib init failed");
        exit(0);
    }
    // Declare camera parameters
    declareParameters();

    // Create camera publisher
    // rqt_image_view can't su1bscribe image msg with sensor_data QoS
    // https://github.com/ros-visualization/rqt/issues/187
    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "galaxy_camera");
    camera_info_manager_ =
        std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url = this->declare_parameter(
        "camera_info_url", "package://ros2_galaxy_camera/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
        camera_info_manager_->loadCameraInfo(camera_info_url);
        camera_info_msg_ = std::make_shared<sensor_msgs::msg::CameraInfo>(camera_info_manager_->getCameraInfo());
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    // Add callback to the set parameter event
    params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&galaxy_camera_node::parametersCallback, this, std::placeholders::_1));

    if(use_hard_trigger){
        createTimestampSub();
    };


    ///打开相机,开始采图并发布图片
    Open(camera_index);
    last_publish_time = rclcpp::Clock().now();
    RCLCPP_INFO(this->get_logger(), "Publishing image!");
}

galaxy_camera_node::~galaxy_camera_node()
{
    //不要阻塞主线程，否则节点不会被启动，即使可以发布数据也有一些节点的功能无法被调用例如动态调参
    while(rclcpp::ok()){
        if(rclcpp::Duration(rclcpp::Clock().now() - last_publish_time).seconds() > 1){
            rclcpp::shutdown();
            throw std::runtime_error("Long time no picture income.");
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    Close();

    RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
}

void galaxy_camera_node::publish_image(void* parameters_package){
    if(debug){
     auto img_delay = rclcpp::Duration(rclcpp::Clock().now()-last_publish_time).seconds()*1000;//ms
     std::cout<<"img_delay:"<<img_delay<<std::endl;
    }
    camera_info_msg_->header.stamp = last_publish_time = rclcpp::Clock().now();
    auto* parameters = static_cast<GX_FRAME_CALLBACK_PARAM*>(parameters_package);
    // image = cv::Mat(cv::Size(parameters->nWidth,parameters->nHeight),CV_8UC1, const_cast<void*>(parameters->pImgBuf)).clone();
    image = cv::Mat(cv::Size(parameters->nWidth,parameters->nHeight),CV_8UC1, const_cast<void*>(parameters->pImgBuf));
    cvtColor(image,image, cv_color_code);
    if(flip_image){
        flip(image,image,-1);
    }
    while(rclcpp::ok()){
        image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        image_msg_->header.frame_id = "camera_optical_frame";
        if(use_hard_trigger){
            std::unique_lock lock(timestamp_lock);
            image_msg_->header.stamp = timestamps.back();
        }
        else{
            image_msg_->header.stamp = camera_info_msg_->header.stamp;
        }
        camera_pub_.publish(image_msg_, camera_info_msg_);
        break;
    }
}

void galaxy_camera_node::declareParameters()
{
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 2;
    param_desc.integer_range[0].step = 0.0001;
    red_blance = this->declare_parameter("red_blance", 1.66, param_desc);
    blue_blance = this->declare_parameter("blue_balance", 1.4844, param_desc);
    green_blance = this->declare_parameter("green_blance", 1, param_desc);
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 30000;
    param_desc.integer_range[0].step = 1;
    exposure_time = this->declare_parameter("exposure_time", 6000, param_desc);
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 16;
    param_desc.integer_range[0].step = 1;
    gain = this->declare_parameter("gain", 6, param_desc);
    camera_index = this->declare_parameter("camera_index", 0, param_desc);
    flip_image = this->declare_parameter("flip_image", false);
    use_hard_trigger = this->declare_parameter("use_hard_trigger", false);
    color_code = this->declare_parameter("color_code", "BG2BGR");
    if(color_code == "BG2BGR"){
        cv_color_code = cv::COLOR_BayerBG2BGR;
    }
    else if(color_code == "GB2BGR"){
        cv_color_code = cv::COLOR_BayerGB2BGR;
    }
    else {
        RCLCPP_ERROR_STREAM(this->get_logger(),"unkonwn color code: " + color_code + ",set default BG2BGR");
        cv_color_code = cv::COLOR_BayerBG2BGR;
    }
}

rcl_interfaces::msg::SetParametersResult galaxy_camera_node::parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool set_success;
    for (const auto & param : parameters) {
    if (param.get_name() == "red_blance") {
        red_blance = param.as_double();
        set_success = SetWhiteBalanceRedChannel(red_blance);
        if (!set_success) {
        result.successful = false;
        result.reason = "Failed to set red_blance";
        }
    } else if (param.get_name() == "blue_balance") {
        blue_blance = param.as_double();
        set_success = SetWhiteBalanceBlueChannel(blue_blance);
        if (!set_success) {
        result.successful = false;
        result.reason = "Failed to set RGB gain";
        }
    } else if (param.get_name() == "green_blance") {
        green_blance = param.as_double();
        set_success = SetWhiteBalanceGreenChannel(green_blance);
        if (!set_success) {
        result.successful = false;
        result.reason = "Failed to set RGB gain";
        }
    } else if (param.get_name() == "exposure_time") {
        exposure_time = param.as_int();
        set_success = SetExposureTime(exposure_time);
        if (!set_success) {
        result.successful = false;
        result.reason = "Failed to set exposure_time";
        }
    } else if (param.get_name() == "gain") {
        gain = param.as_int();
        set_success = SetGain(gain);
        if (!set_success) {
        result.successful = false;
        result.reason = "Failed to set gain";
        }
    } else if(param.get_name() == "use_hard_trigger"){
        use_hard_trigger = param.as_bool();
        Close();
        Open(camera_index);
        if(use_hard_trigger){
            createTimestampSub();
        }
        else{
            destroyTimestampSub();
        }
        result.successful = true;
    } else if(param.get_name() == "flip_image"){
        flip_image = param.as_bool();
        result.successful = true;
    } else if(param.get_name() == "color_code"){
        if(color_code != "BG2BGR" && color_code != "GB2BGR"){
            result.successful = false;
            result.reason = "Unknown parameter: " + param.get_name();
        }
        else{
            color_code = param.as_string();
            result.successful = true;
            if(color_code == "BG2BGR"){
                cv_color_code = cv::COLOR_BayerBG2BGR;
            }
            else if(color_code == "GB2BGR"){
                cv_color_code = cv::COLOR_BayerGB2BGR;
            }
            else {
                RCLCPP_ERROR_STREAM(this->get_logger(),"unkonwn color code: " + color_code + ",set default BG2BGR");
                cv_color_code = cv::COLOR_BayerBG2BGR;
            }
        }
    } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
    }
    }
    return result;
}

void galaxy_camera_node::createTimestampSub(){
    timestamp_sub_ = this->create_subscription<builtin_interfaces::msg::Time>(
        "/port_timestamp", rclcpp::SensorDataQoS(),
        [this](const builtin_interfaces::msg::Time::SharedPtr msg){
            std::unique_lock lock(timestamp_lock);
            timestamps.push(*msg);
            if(timestamps.size() > 3){
                timestamps.pop();
            }
        }
    );
}

void galaxy_camera_node::destroyTimestampSub(){
    timestamp_sub_.reset();
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_galaxy_camera::galaxy_camera_node)
