/**
 * @file robotiq_gripper_interface.cpp
 * @author Adrian Mueller (adrian.mueller@study.thws.de)
 * @brief 
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ur_ros_driver/robotiq_gripper_interface.hpp>

RobotiqGripperInterface::RobotiqGripperInterface()
{

}

bool RobotiqGripperInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
    this->initParameter(robot_hw_nh);
    this->robotiq_gripper_ = new ur_rtde::RobotiqGripper(robot_ip_, 63352, true);
    
    this->robotiq_gripper_->connect();
    this->robotiq_gripper_->activate();

    this->robotiq_gripper_->autoCalibrate();
    this->robotiq_gripper_->close();

    this->robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_MM);
    this->robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::SPEED, ur_rtde::RobotiqGripper::UNIT_PERCENT);
    this->robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::FORCE, ur_rtde::RobotiqGripper::UNIT_PERCENT);
    
    this->initServices(robot_hw_nh);
    this->initTopics(robot_hw_nh);

    this->gripper_thread_ = new std::thread(&RobotiqGripperInterface::run,this);
    
    return true;
}

bool RobotiqGripperInterface::initParameter(ros::NodeHandle& robot_hw_nh)
{
    if (!robot_hw_nh.getParam("robot_ip", robot_ip_))
    {
        ROS_ERROR_STREAM("Required parameter robot_ip not given. gripper");
        return false;
    }
    return true;
}

void RobotiqGripperInterface::initServices(ros::NodeHandle& robot_hw_nh)
{
    set_gripper_srv_ = robot_hw_nh.advertiseService("/set_gripper", &RobotiqGripperInterface::set_gripper, this);
    calib_gripper_srv_ = robot_hw_nh.advertiseService("/calib_gripper", &RobotiqGripperInterface::calib_gripper, this);
    get_gripper_calib_srv_ = robot_hw_nh.advertiseService("/get_gripper_calib", &RobotiqGripperInterface::get_gripper_calib, this);
}

void RobotiqGripperInterface::initTopics(ros::NodeHandle& robot_hw_nh)
{
    gripper_info_pub_.reset(new realtime_tools::RealtimePublisher<ur_ros_driver::GripperInfo>(robot_hw_nh, "/gripper_infos", 100));
}

void RobotiqGripperInterface::read(const ros::Time& time, const ros::Duration& period)
{
    read_gripper_infos();
}

void RobotiqGripperInterface::write(const ros::Time& time, const ros::Duration& period)
{
    publish_gripper_infos();
}

bool RobotiqGripperInterface::set_gripper(ur_ros_driver::SetGripperRequest& req,
                                          ur_ros_driver::SetGripperResponse& res)
{
    try{
        if(!robotiq_gripper_->isConnected() && !robotiq_gripper_->isActive())
        {
            robotiq_gripper_->connect();
            robotiq_gripper_->activate();
        }

        bool error;

        switch (req.position_unit)
        {
        case 0:
            robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_PERCENT);
            if(req.position > 100.0 || req.position < 0.0){error = true;}
            break;
        case 1:
            robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_MM);
            break;
        case 3:
            robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_DEVICE);
            if(req.position > 255 || req.position < 0){error = true;}
            break;
        default:
            error = true;
            break;
        }

        if(!error)
        {
            res.success = robotiq_gripper_->move(req.position,req.speed,req.force,ur_rtde::RobotiqGripper::WAIT_FINISHED);
            res.eObjectStatus = robotiq_gripper_->objectDetectionStatus();
        }
        else
        {
            res.success = false;
            ROS_ERROR("gripper position values out of range");
        }
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
        res.success = false;
        return 1;
    }

    return true;
}

bool RobotiqGripperInterface::calib_gripper(std_srvs::TriggerRequest& req,
                                            std_srvs::TriggerResponse& res)
{
    try{
        if(!robotiq_gripper_->isConnected() && !robotiq_gripper_->isActive())
        {
            robotiq_gripper_->connect();
            robotiq_gripper_->activate();
        }

        robotiq_gripper_->autoCalibrate();
        res.success = true;
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
        res.success = false;
        return 1;
    }
    

    return true;
}

bool RobotiqGripperInterface::get_gripper_calib(ur_ros_driver::GetGripperCalibRequest& req,
                                                ur_ros_driver::GetGripperCalibResponse& res)
{
    try{
        robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_MM);
        res.mm_max = robotiq_gripper_->getOpenPosition();
        res.mm_current = robotiq_gripper_->getCurrentPosition();
        res.mm_min = robotiq_gripper_->getClosedPosition();

        robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_PERCENT);
        res.per_max = robotiq_gripper_->getOpenPosition();
        res.per_current = robotiq_gripper_->getCurrentPosition();
        res.per_min = robotiq_gripper_->getClosedPosition();

        robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_DEVICE);
        res.dev_max = robotiq_gripper_->getOpenPosition();
        res.dev_current = robotiq_gripper_->getCurrentPosition();
        res.dev_min = robotiq_gripper_->getClosedPosition();
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
        return 1;
    }
    return true;
}

void RobotiqGripperInterface::publish_gripper_infos()
{
    if(gripper_info_pub_)
    {
        if (gripper_info_pub_->trylock())
        {
            gripper_info_pub_->msg_ = gripper_info_;
            gripper_info_pub_->unlockAndPublish();
        }
    }
}

void RobotiqGripperInterface::read_gripper_infos()
{
    gripper_info_.stamp = ros::Time::now();
    robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_MM);
    gripper_info_.position_mm = robotiq_gripper_->getCurrentPosition();
    robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_PERCENT);
    gripper_info_.position_per = robotiq_gripper_->getCurrentPosition();
    robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_DEVICE);
    gripper_info_.position_device = robotiq_gripper_->getCurrentPosition();

    gripper_info_.eFaultCode = robotiq_gripper_->faultStatus();
    gripper_info_.eObjectStatus = robotiq_gripper_->objectDetectionStatus();

    gripper_info_.connected = robotiq_gripper_->isConnected();
    gripper_info_.activated = robotiq_gripper_->isActive();

}

void RobotiqGripperInterface::run()
{
    ROS_GREEN_STREAM("Gripper thread is started");
    ros::Rate rate(100);

    ros::Time timestamp;
    ros::Duration period;
    auto stopwatch_last = std::chrono::steady_clock::now();
    auto stopwatch_now = stopwatch_last;

    while (ros::ok())
    {
        try{
            // Get current time and elapsed time since last read
            timestamp = ros::Time::now();
            stopwatch_now = std::chrono::steady_clock::now();
            period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
            stopwatch_last = stopwatch_now;

            // Receive current state from robot
            this->read(timestamp, period);
            this->write(timestamp, period);
            
        }
        catch(std::exception& e) {
            ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());

        }
        catch(...) {
            ROS_ERROR_STREAM_THROTTLE(5,"Exception of unknown type!");
        }

        rate.sleep();
    }
}