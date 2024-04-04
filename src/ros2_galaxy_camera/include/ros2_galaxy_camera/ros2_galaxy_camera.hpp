/*** 
 * @Author: gaoyuan
 * @Date: 2023-04-07 11:29:07
 * @LastEditors: gaoyuan
 * @LastEditTime: 2023-07-18 17:31:34
 */
#pragma once
#include "GxIAPI.h"
#include <opencv2/opencv.hpp>

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/camera_publisher.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/duration.hpp>

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <unordered_set>
#include <functional>
#include <mutex>

namespace ros2_galaxy_camera
{
void HandleCameraDeviceOfflineEvent(void* parameter);
void HandlePictureIncomeEvent(void* parameters_package);

class galaxy_camera_node : public rclcpp::Node
{

public:
    explicit galaxy_camera_node(const rclcpp::NodeOptions & options);

    ~galaxy_camera_node();

    /// 相机离线事件处理
    friend void HandleCameraDeviceOfflineEvent(void* parameter);
    /// 图片到达事件处理
    friend void HandlePictureIncomeEvent(void* parameters_package);

    void publish_image(void* parameters_package);

private:
    void declareParameters();

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> & parameters);

    void createTimestampSub();

    void destroyTimestampSub();

private:    
    sensor_msgs::msg::Image::SharedPtr image_msg_;

    image_transport::CameraPublisher camera_pub_;

    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_;

    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr timestamp_sub_;
    std::queue<builtin_interfaces::msg::Time> timestamps;
    std::mutex timestamp_lock;
    
    rclcpp::Time last_publish_time;

    std::string camera_name_;

    int camera_index;

private:
    bool debug =1;

    cv::Mat image;

        ///是否翻转图像
    bool flip_image;

    ///颜色编码方式
    std::string color_code;
    cv::ColorConversionCodes cv_color_code;

    /// 设备句柄
    void* DeviceHandle {nullptr};
    /// 设备离线事件句柄
    void* DeviceOfflineHandle {nullptr};
    
    //是否采用硬触发
    bool use_hard_trigger;

    /// 设备是否已经被打开
    bool opened {false};

    ///图像rgb平衡
    double red_blance;
    double green_blance;
    double blue_blance;

    ///曝光
    int exposure_time;

    ///增益
    int gain;

private:

    /**
     * @brief 开启指定相机设备，注意，GalaxySDK中开启设备的该方法中设备索引是从1开始编号的
     */
    void Open(uint32_t CameraIndex){
        // 操作执行结果
		GX_STATUS operation_result;

		// 获取设备列表
		uint32_t device_count = 0;
		operation_result = GXUpdateDeviceList(&device_count, 500);
		if (operation_result != GX_STATUS_LIST::GX_STATUS_SUCCESS)
		{
			throw std::runtime_error("CameraDevice::Open Failed to Query Device List.");
		}
		if (device_count <= 0)
		{
			throw std::runtime_error("CameraDevice::Open No Camera Detected.");
		}
		if (device_count <= (CameraIndex))
		{
			std::stringstream message;
			message << "CameraDevice::Open Invalid Device Index: " << CameraIndex
				<< " , Camera Counts: " << device_count;
			throw std::runtime_error(message.str());
		}

		// 开启指定设备，注意，GalaxySDK中开启设备的该方法中设备索引是从1开始编号的
		operation_result = GXOpenDeviceByIndex(CameraIndex + 1, &DeviceHandle);
		if (operation_result != GX_STATUS_LIST::GX_STATUS_SUCCESS)
		{
			throw std::runtime_error("CameraDevice::Open Failed to Open Device.");
		}

        // 设置相机为硬触发模式
        if(use_hard_trigger){

            // 确保触发模式为开启状态
            operation_result = GXSetEnum(DeviceHandle,GX_ENUM_TRIGGER_MODE,GX_TRIGGER_MODE_ON);
            if (operation_result != GX_STATUS_LIST::GX_STATUS_SUCCESS)
            {
                throw std::runtime_error("CameraDevice::Open Failed to turn trigger mode on.");
            }
            
            // 关闭图像采集帧率控制模式
            operation_result = GXSetEnum(DeviceHandle,GX_ENUM_ACQUISITION_FRAME_RATE_MODE,GX_ACQUISITION_FRAME_RATE_MODE_OFF);
            if (operation_result != GX_STATUS_LIST::GX_STATUS_SUCCESS)
            {
                throw std::runtime_error("CameraDevice::Open Failed to turn acquisition frame rate mode off.");
            }
            
            // 设置触发信号源
            operation_result = GXSetEnum(DeviceHandle,GX_ENUM_TRIGGER_SOURCE,GX_TRIGGER_SOURCE_LINE2);
            if (operation_result != GX_STATUS_LIST::GX_STATUS_SUCCESS)
            {
                throw std::runtime_error("CameraDevice::Open Failed to set trigger source.");
            }

            // 设置触发激活方式为上升沿
            operation_result = GXSetEnum(DeviceHandle , GX_ENUM_TRIGGER_ACTIVATION,GX_TRIGGER_ACTIVATION_RISINGEDGE);
            if (operation_result != GX_STATUS_LIST::GX_STATUS_SUCCESS)
            {
                throw std::runtime_error("CameraDevice::Open Failed to set trigger activation.");
            }
        }
        
		// 注册采集回调
		operation_result = GXRegisterCaptureCallback(DeviceHandle, this, 
                reinterpret_cast<GXCaptureCallBack>(HandlePictureIncomeEvent));
		if (operation_result != GX_STATUS_LIST::GX_STATUS_SUCCESS)
		{
			throw std::runtime_error("MatAcquisitor::Start Failed to Register Capture Callback.");
		}

		// 注册离线回调
		operation_result = GXRegisterDeviceOfflineCallback(DeviceHandle, this, 
                HandleCameraDeviceOfflineEvent,&DeviceOfflineHandle);
		if (operation_result != GX_STATUS_LIST::GX_STATUS_SUCCESS)
		{
			throw std::runtime_error("CameraDevice::Open Failed to Register Offline Callback.");
		}
        bool set_success;
        set_success = SetWhiteBalanceRedChannel(red_blance);
        if (!set_success) {
            RCLCPP_ERROR(this->get_logger(),"Failed to set red_blance");
            exit(0);
        }
        set_success = SetWhiteBalanceBlueChannel(blue_blance);
        if (!set_success) {
            RCLCPP_ERROR(this->get_logger(),"Failed to set RGB gain");
            exit(0);
        }
        set_success = SetWhiteBalanceGreenChannel(green_blance);
        if (!set_success) {
            RCLCPP_ERROR(this->get_logger(),"Failed to set RGB gain");    
            exit(0);
        }
        set_success = SetExposureTime(exposure_time);
        if (!set_success) {
            RCLCPP_ERROR(this->get_logger(),"Failed to set exposure_time");
            exit(0);
        }
        set_success = SetGain(gain);
        if (!set_success) {
            RCLCPP_ERROR(this->get_logger(),"Failed to set gain");
            exit(0);
        }
		// 发送命令开始采集
		operation_result = GXSendCommand(DeviceHandle, GX_COMMAND_ACQUISITION_START);
		if (operation_result != GX_STATUS_LIST::GX_STATUS_SUCCESS)
		{
			throw std::runtime_error("CameraDevice::Open Failed to Start Acquisition.");
		}

		opened = true;
    };

    /**
     * @brief 关闭相机
     * @details
     *  ~ 不会触发相机离线事件。
     */
    void Close(){
		if (DeviceHandle)
		{
			GXSendCommand(DeviceHandle, GX_COMMAND_ACQUISITION_STOP);

			GXUnregisterCaptureCallback(DeviceHandle);
			GXUnregisterDeviceOfflineCallback(DeviceHandle, DeviceOfflineHandle);

			GXCloseDevice(DeviceHandle);
			DeviceHandle = nullptr;
		}
    };

    /**
     * @brief 设置曝光时间
     * @param value 曝光时间，单位为微秒(us)
     * @pre 设备已经打开
     * @retval true 当相机已经开启
     * @retval false 当相机未被开启
     */
    bool SetExposureTime(double value){
		if (DeviceHandle && GXSetFloat(DeviceHandle, GX_FLOAT_EXPOSURE_TIME, value) 
            == GX_STATUS_LIST::GX_STATUS_SUCCESS)
		{
			return true;
		}
		return false;
    };


    /**
     * @brief 设置增益
     * @param value 增益，单位为db
     * @pre 设备已经打开
     * @return 是否操作成功，操作成功则返回true，失败则返回false
     */
    bool SetGain(double value){
        if (DeviceHandle && GXSetFloat(DeviceHandle, GX_FLOAT_GAIN, value) 
            == GX_STATUS_LIST::GX_STATUS_SUCCESS)
		{
			return true;
		}
		return false;
    };

    /**
     * @brief 设置白平衡通道红色值
     * @param value 白平衡值
     * @retval true 当操作成功
     * @retval false 当相机未开启或操作失败
     */
    bool SetWhiteBalanceRedChannel(double value){
		if (DeviceHandle)
		{
			GX_STATUS operation_result;
			operation_result = GXSetEnum(DeviceHandle, 
                GX_ENUM_BALANCE_RATIO_SELECTOR,
                GX_BALANCE_RATIO_SELECTOR_RED);
			if (operation_result != GX_STATUS_LIST::GX_STATUS_SUCCESS)
			{
				return false;
			}
			operation_result = GXSetFloat(DeviceHandle, GX_FLOAT_BALANCE_RATIO, value);
			if (operation_result != GX_STATUS_LIST::GX_STATUS_SUCCESS)
			{
				return false;
			}

			return true;
		}
		return false;
    };

    /**
     * @brief 设置白平衡通道绿色值
     * @param value 白平衡值
     * @pre 设备已经打开
     * @retval true 当相机已经开启
     * @retval false 当相机未开启或操作失败
     */
    bool SetWhiteBalanceGreenChannel(double value){
        if (DeviceHandle)
		{
			GX_STATUS operation_result;
			operation_result = GXSetEnum(DeviceHandle, 
                GX_ENUM_BALANCE_RATIO_SELECTOR,
                GX_BALANCE_RATIO_SELECTOR_GREEN);
			if (operation_result != GX_STATUS_LIST::GX_STATUS_SUCCESS)
			{
				return false;
			}
			operation_result = GXSetFloat(DeviceHandle, 
                GX_FLOAT_BALANCE_RATIO, value);
			if (operation_result != GX_STATUS_LIST::GX_STATUS_SUCCESS)
			{
				return false;
			}

			return true;
		}
		return false;
    };

    /**
     * @brief 设置白平衡通道蓝色值
     * @param value 白平衡值
     * @pre 设备已经打开
     * @retval true 当相机已经开启
     * @retval false 当相机未开启或操作失败
     */
    bool SetWhiteBalanceBlueChannel(double value){
        if (DeviceHandle)
		{
			GX_STATUS operation_result;
			operation_result = GXSetEnum(DeviceHandle, 
                GX_ENUM_BALANCE_RATIO_SELECTOR,
                GX_BALANCE_RATIO_SELECTOR_BLUE);
			if (operation_result != GX_STATUS_LIST::GX_STATUS_SUCCESS)
			{
				return false;
			}
			operation_result = GXSetFloat(DeviceHandle, 
                GX_FLOAT_BALANCE_RATIO, value);
			if (operation_result != GX_STATUS_LIST::GX_STATUS_SUCCESS)
			{
				return false;
			}

			return true;
		}
		return false;
    };

    bool SetAutoWhiteBalance(){
        GX_STATUS operation_result;
        operation_result = GXSetEnum(DeviceHandle,
            GX_ENUM_BALANCE_WHITE_AUTO,
            GX_BALANCE_WHITE_AUTO_CONTINUOUS);
        if (operation_result != GX_STATUS_LIST::GX_STATUS_SUCCESS)
        {
            return false;
        }
        return true;
    };

    };

    /// 相机离线事件处理
    void HandleCameraDeviceOfflineEvent(void* parameter){
        throw std::runtime_error("CameraDevice Offline");
    }

    /// 图片到达事件处理
    void HandlePictureIncomeEvent(void* parameters_package){
        auto* parameters =  static_cast<GX_FRAME_CALLBACK_PARAM*>(parameters_package);
        auto* target = static_cast<galaxy_camera_node*>(parameters->pUserParam);
        target->publish_image(parameters_package);
    }

}
