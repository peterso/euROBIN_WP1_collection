/**
 * @file dashboard_client.cpp
 * @author Adrian Mueller (adrian.mueller@study.thws.de)
 * @brief 
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ur_ros_driver/dashboard_client.hpp>

DashboardClient::DashboardClient()
{
}

bool DashboardClient::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{

    if(!this->initParameter(robot_hw_nh)){return false;}
    this->rtde_io_ = new ur_rtde::RTDEIOInterface(robot_ip_);
    this->rtde_dashboard_ = new ur_rtde::DashboardClient(robot_ip_);

    this->initServices(robot_hw_nh);
    this->initTopics(robot_hw_nh);

    this->rtde_dashboard_->connect(2000U);
    this->dashboard_thread_ = new std::thread(&DashboardClient::run,this);

    return true;
}

void DashboardClient::initServices(ros::NodeHandle& robot_hw_nh)
{
    // load_URprogram_srv_ =  robot_hw_nh.advertiseService("/set_cart_target", &URHardwareInterface::set_cart_target, this);
    power_srv_ =  robot_hw_nh.advertiseService("/power", &DashboardClient::power, this);
    brake_release_srv_ =  robot_hw_nh.advertiseService("/brake_release", &DashboardClient::brake_release, this);
    unlock_protective_stop_srv_ =  robot_hw_nh.advertiseService("/unlock_protective_stop", &DashboardClient::unlock_protective_stop, this);
    restart_safety_srv_ =  robot_hw_nh.advertiseService("/restart_safety", &DashboardClient::restart_safety, this);
    // play_ =  robot_hw_nh.advertiseService("/dashboard/play", &DashboardClient::play, this);
    // pause_ =  robot_hw_nh.advertiseService("/dashboard/pause", &DashboardClient::pause, this);
    // stop_ =  robot_hw_nh.advertiseService("/dashboard/stop", &DashboardClient::stop, this);
    
    speed_slider_srv_ = robot_hw_nh.advertiseService("/set_speed_slider", &DashboardClient::set_speed_slider,this);
}

bool DashboardClient::initParameter(ros::NodeHandle& robot_hw_nh)
{
    if (!robot_hw_nh.getParam("robot_ip", this->robot_ip_))
    {
        ROS_ERROR_STREAM("Required parameter robot_ip not given. dashboard");
        return false;
    }
    return true;
}

void DashboardClient::initTopics(ros::NodeHandle& robot_hw_nh)
{
    dashboard_info_pub_.reset(new realtime_tools::RealtimePublisher<ur_ros_driver::DashboardInfo>(robot_hw_nh, "/dashboard_info", 100));
}

void DashboardClient::check_state()
{
    if(!rtde_dashboard_->isConnected())
    {
        this->rtde_dashboard_->connect(2000U);
    }
}

void DashboardClient::read(const ros::Time& time, const ros::Duration& period)
{
    this->read_dashboard_infos();
}

void DashboardClient::write(const ros::Time& time, const ros::Duration& period)
{
    this->publish_dashboard_infos();
}

void DashboardClient::read_dashboard_infos()
{
    dashboard_info_msg_.isConnected = rtde_dashboard_->isConnected();
    // dashboard_info_msg_.running = rtde_dashboard_->running();
    // dashboard_info_msg_.isInRemoteControl = rtde_dashboard_->isInRemoteControl();

    // dashboard_info_msg_.robotmode = rtde_dashboard_->robotmode();
    // dashboard_info_msg_.programState = rtde_dashboard_->programState();
    // dashboard_info_msg_.loadedProgram = rtde_dashboard_->getLoadedProgram();
    rtde_dashboard_->popup(dashboard_info_msg_.popup);
    if(dashboard_info_msg_.popup != "")
    {
        ROS_MAGENTA_STREAM_THROTTLE(5,dashboard_info_msg_.popup);
    }

    // dashboard_info_msg_.safetymode = rtde_dashboard_->safetymode();
    // dashboard_info_msg_.safetystatus = rtde_dashboard_->safetystatus();

    // dashboard_info_msg_.getSerialNumber = serial_number_; 
    // dashboard_info_msg_.getRobotModel = robot_model_; 
    // dashboard_info_msg_.polyscopeVersion = polyscope_version_; 
}

void DashboardClient::publish_dashboard_infos()
{
    if (dashboard_info_pub_)
    {
        if (dashboard_info_pub_->trylock())
        {
            dashboard_info_pub_->msg_ = dashboard_info_msg_;
            dashboard_info_pub_->unlockAndPublish();
        }
    }
}

bool DashboardClient::power(ur_ros_driver::PowerRequest& req,ur_ros_driver::PowerResponse& res)
{
    try{
        if(req.IO)
        {
            rtde_dashboard_->powerOn();
        }
        else{
            rtde_dashboard_->powerOff();
        }
        res.success = true;
        return true;
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
    }
    return false;
}

bool DashboardClient::brake_release(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
    try{
        rtde_dashboard_->brakeRelease();
        res.success = true;
        return true;
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
    }
    return false;
}

bool DashboardClient::unlock_protective_stop(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
    try{
        rtde_dashboard_->unlockProtectiveStop();
        res.success = true;
        return true;
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
    }
    return false;
}

bool DashboardClient::play(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
    try{
        rtde_dashboard_->play();
        res.success = true;
        return true;
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
        res.success = false;
    }
    return false;
}

bool DashboardClient::pause(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
    try{
        rtde_dashboard_->pause();
        res.success = true;
        return true;
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
    }
    return false;
}

bool DashboardClient::stop(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
    try{
        rtde_dashboard_->stop();
        res.success = true;
        return true;
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
    }
    return false;
}

bool DashboardClient::restart_safety(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
    try{
        rtde_dashboard_->closeSafetyPopup();
        rtde_dashboard_->brakeRelease();
        res.success = true;
        return true;
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
        res.success = false;
    }
    return false;
}

bool DashboardClient::set_speed_slider(ur_ros_driver::SetSpeedSliderRequest& req,
                                       ur_ros_driver::SetSpeedSliderResponse& res)
{
    if(req.slider >= 0 && req.slider <= 2)
    {
        res.success = rtde_io_->setSpeedSlider(req.slider);
    }
    else
    {
        ROS_WARN_STREAM("Values for speed slider out of range");
        res.success = false;
    }
    return true;
}

bool DashboardClient::isInRemoteControl()
{
    return rtde_dashboard_->isInRemoteControl();
}

void DashboardClient::powerOn()
{
    rtde_dashboard_->powerOn();

}

void DashboardClient::brakeRelease()
{
    rtde_dashboard_->brakeRelease();
}

void DashboardClient::run()
{
    ROS_GREEN_STREAM("Dashboard thread is started");
    ros::Rate rate(50);
    ros::Time timestamp;
    ros::Duration period;
    auto stopwatch_last = std::chrono::steady_clock::now();
    auto stopwatch_now = stopwatch_last;

    //dashboard_client.reset(new DashboardClient);

    while (ros::ok())
    {
        try{
            // Get current time and elapsed time since last read
            timestamp = ros::Time::now();
            stopwatch_now = std::chrono::steady_clock::now();
            period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
            stopwatch_last = stopwatch_now;

            // Receive current state from robot
            this->check_state();
            this->read(timestamp, period);
            this->write(timestamp, period);
        }
        catch(std::exception& e) {
            ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
            ros::Duration(1).sleep();
        }
        catch(...) {
            ROS_ERROR_STREAM_THROTTLE(5,"Exception of unknown type!");
            ros::Duration(1).sleep();
        }
        
        rate.sleep();
    }
}