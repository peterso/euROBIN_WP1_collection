/**
 * @file robotiq_gripper_interface.hpp
 * @author Adrian Mueller (adrian.mueller@study.thws.de)
 * @brief 
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ros/ros.h>

#include <hardware_interface/robot_hw.h>

#include <realtime_tools/realtime_publisher.h>

#include <ur_rtde/robotiq_gripper.h>

#include <std_srvs/Trigger.h>
#include <ur_ros_driver/GripperInfo.h>
#include <ur_ros_driver/GetGripperCalib.h>
#include <ur_ros_driver/SetGripper.h>

#include <ur_ros_driver/ros_color_stream.hpp>

class RobotiqGripperInterface : public hardware_interface::RobotHW
{
private:
    std::string robot_ip_;
    ur_rtde::RobotiqGripper* robotiq_gripper_;

    std::thread* gripper_thread_;

    std::unique_ptr<realtime_tools::RealtimePublisher<ur_ros_driver::GripperInfo>> gripper_info_pub_;

    ros::ServiceServer set_gripper_srv_;
    ros::ServiceServer calib_gripper_srv_;
    ros::ServiceServer get_gripper_calib_srv_;

    ur_ros_driver::GripperInfo gripper_info_;

public:

    /**
     * @brief Construct a new Robotiq Gripper Interface object
     * 
     * @param nh 
     * @param robot_ip 
     */
    RobotiqGripperInterface();

    virtual ~RobotiqGripperInterface() = default;

    virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
    bool initParameter(ros::NodeHandle& robot_hw_nh);
    void initServices(ros::NodeHandle& robot_hw_nh);
    void initTopics(ros::NodeHandle& robot_hw_nh);

    void run();
    virtual void read(const ros::Time& time, const ros::Duration& period) override;
    virtual void write(const ros::Time& time, const ros::Duration& period) override;

    bool set_gripper(ur_ros_driver::SetGripperRequest& req,
                     ur_ros_driver::SetGripperResponse& res);

    bool calib_gripper(std_srvs::TriggerRequest& req,
                       std_srvs::TriggerResponse& res);
                       
    bool get_gripper_calib(ur_ros_driver::GetGripperCalibRequest& req,
                           ur_ros_driver::GetGripperCalibResponse& res);
    
    void publish_gripper_infos();
    void read_gripper_infos();
};

