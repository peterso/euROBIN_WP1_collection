/**
 * @file hw_interface.hpp
 * @author Adrian Mueller (adrian.mueller@study.thws.de)
 * @brief 
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */


//TODO: ur_msgs hinzufügen http://docs.ros.org/en/kinetic/api/ur_msgs/html/index-msg.html

#include <ros/ros.h>
#include <thread>  

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

#include <controller_manager/controller_manager.h>

#include <realtime_tools/realtime_publisher.h>

#include <ur_ros_driver/dashboard_client.hpp>
#include <ur_ros_driver/robotiq_gripper_interface.hpp>

#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>

#include <ur_ros_driver/SetCartTarget.h>
#include <ur_ros_driver/SetRelCartTarget.h>
#include <ur_ros_driver/SetJointTarget.h>
#include <ur_ros_driver/SetTrajectory.h>
#include <ur_ros_driver/SetFreedrive.h>
#include <ur_ros_driver/SetForceTarget.h>
#include <ur_ros_driver/SetContactTarget.h>
#include <ur_ros_driver/StartJog.h>
#include <ur_ros_driver/JogControl.h>
#include <ur_ros_driver/Log.h>
#include <ur_ros_driver/SetPayload.h>
#include <ur_ros_driver/GetTimeStamp.h>



#include <ur_ros_driver/ros_color_stream.hpp>

/**
 * @brief Class that provides the real time data of the robot
 * 
 */
class URHardwareInterface : public hardware_interface::RobotHW
{
private:

    ros::NodeHandle nh_;

    DashboardClient* dashboard_;
    RobotiqGripperInterface* gripper_;

    std::string robot_ip_ = "172.31.1.200";
    ur_rtde::RTDEReceiveInterface* rtde_receive_;
    ur_rtde::RTDEControlInterface* rtde_control_;

    bool robot_status;

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;
    
    std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TransformStamped>> tcp_pose_pub_;
    std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>> wrench_pub_;
    //realtime_tools::RealtimePublisher<sensor_msgs::JointState> joint_value_pub_;

    ros::Subscriber jog_conroll_sub_;

    ros::ServiceServer set_cart_target_srv_;
    ros::ServiceServer set_rel_cart_target_srv_;
    ros::ServiceServer set_joint_target_srv_;
    ros::ServiceServer set_rel_joint_target_srv_;
    ros::ServiceServer set_trajectory_srv_;
    ros::ServiceServer set_freedive_srv_;
    ros::ServiceServer set_force_target_srv_;
    ros::ServiceServer set_contact_target_srv_;
    ros::ServiceServer start_jog_srv_;
    ros::ServiceServer zero_ftsensor_srv_;
    ros::ServiceServer log_srv_;
    ros::ServiceServer set_payload_srv_;
    ros::ServiceServer get_timestamp_srv_;

    geometry_msgs::Transform tcp_pose_;
    //sensor_msgs::JointState joint_value_;
    geometry_msgs::Wrench wrench_;
    std::vector<std::string> joint_names_;
    std::array<double, 6> joint_positions_;
    std::array<double, 6> joint_velocities_;
    std::array<double, 6> joint_efforts_;

    void transform_to_angelaxis(geometry_msgs::Transform& transform, std::vector<double>& angleaxis);

public:

    /**
     * @brief Construct a new Receive Interface object
     * 
     */
    URHardwareInterface();

    virtual ~URHardwareInterface() = default;

    virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

    bool initParameter(ros::NodeHandle& robot_hw_nh);
    void initInterfaces(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
    void initROSInterfaces(ros::NodeHandle& robot_hw_nh);
    void initServices(ros::NodeHandle& robot_hw_nh);
    void initTopics(ros::NodeHandle& robot_hw_nh);

    bool start_robot();
    void check_state();

    virtual void read(const ros::Time& time, const ros::Duration& period) override;

    virtual void write(const ros::Time& time, const ros::Duration& period) override;

    bool set_cart_target(ur_ros_driver::SetCartTargetRequest& req, ur_ros_driver::SetCartTargetResponse& res);
    bool set_rel_cart_target(ur_ros_driver::SetRelCartTargetRequest& req, ur_ros_driver::SetRelCartTargetResponse& res);
    bool set_joint_target(ur_ros_driver::SetJointTargetRequest& req, ur_ros_driver::SetJointTargetResponse& res);
    bool set_rel_joint_target(ur_ros_driver::SetJointTargetRequest& req, ur_ros_driver::SetJointTargetResponse& res);
    bool set_trajectory(ur_ros_driver::SetTrajectoryRequest& req, ur_ros_driver::SetTrajectoryResponse& res);
    bool set_force_target(ur_ros_driver::SetForceTargetRequest& req, ur_ros_driver::SetForceTargetResponse& res);
    bool set_contact_target(ur_ros_driver::SetContactTargetRequest& req,ur_ros_driver::SetContactTargetResponse& res);
    bool set_freedive(ur_ros_driver::SetFreedriveRequest& req, ur_ros_driver::SetFreedriveResponse& res);
    bool start_jog(ur_ros_driver::StartJogRequest& req, ur_ros_driver::StartJogResponse& res);
    bool zero_ftsensor(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
    bool log(ur_ros_driver::LogRequest& req, ur_ros_driver::LogResponse& res);
    bool set_payload(ur_ros_driver::SetPayloadRequest& req, ur_ros_driver::SetPayloadResponse& res);
    bool get_timestamp(ur_ros_driver::GetTimeStampRequest& req, ur_ros_driver::GetTimeStampResponse& res);

    void jog_control_callback(const ur_ros_driver::JogControl& msg);

    void publish_tcp_pose();
    void publish_wrench();

    void read_connection_state();
    void read_tcp_pose();
    void read_joint_values();
    void read_wrench();

};

