/**
 * @file dashboard_client.hpp
 * @author Adrian Mueller (adrian.mueller@study.thws.de)
 * @brief 
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <ros/ros.h>

#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/rtde_io_interface.h>

#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>

#include <std_srvs/Trigger.h>
#include <ur_ros_driver/DashboardInfo.h>
#include <ur_ros_driver/SetSpeedSlider.h>
#include <ur_ros_driver/Power.h>

#include <ur_ros_driver/ros_color_stream.hpp>

class DashboardClient : public hardware_interface::RobotHW
{
private:
    ros::NodeHandle nh_;
    std::string robot_ip_;
    ur_rtde::DashboardClient* rtde_dashboard_;
    ur_rtde::RTDEIOInterface* rtde_io_;

    std::thread* dashboard_thread_;

    std::unique_ptr<realtime_tools::RealtimePublisher<ur_ros_driver::DashboardInfo>> dashboard_info_pub_;

    //Services DashboardClient
    //ros::ServiceServer load_URprogram_srv_;
    ros::ServiceServer power_srv_;
    ros::ServiceServer brake_release_srv_;
    ros::ServiceServer unlock_protective_stop_srv_;
    ros::ServiceServer restart_safety_srv_;
    ros::ServiceServer play_;
    ros::ServiceServer pause_;
    ros::ServiceServer stop_;

    //Services RTDEIOInterface
    ros::ServiceServer speed_slider_srv_;

    std::string serial_number_,robot_model_,polyscope_version_;
    ur_ros_driver::DashboardInfo dashboard_info_msg_;


    void read_dashboard_infos();
    void publish_dashboard_infos();

    void run();

public:

    DashboardClient();

    virtual ~DashboardClient() = default;

    virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
    bool initParameter(ros::NodeHandle& robot_hw_nh);
    void initServices(ros::NodeHandle& robot_hw_nh);
    void initTopics(ros::NodeHandle& robot_hw_nh);

    void check_state();
    virtual void read(const ros::Time& time, const ros::Duration& period) override;
    virtual void write(const ros::Time& time, const ros::Duration& period) override;

    bool power(ur_ros_driver::PowerRequest& req,ur_ros_driver::PowerResponse& res);
    bool brake_release(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
    bool unlock_protective_stop(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
    bool restart_safety(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
    bool set_speed_slider(ur_ros_driver::SetSpeedSliderRequest& req,ur_ros_driver::SetSpeedSliderResponse& res);
    bool play(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
    bool pause(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
    bool stop(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);

    bool isInRemoteControl();
    bool closePopup();

    void powerOn();
    void powerOff();
    void brakeRelease();
    void unlockProtectiveStop();
};
