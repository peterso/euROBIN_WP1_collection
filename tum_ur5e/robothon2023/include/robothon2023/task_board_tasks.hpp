/**
 * @file task_board_tasks.hpp
 * @authors Adrian Müller (adrian.mueller@study.thws.de), 
 *          Maximilian Hornauer (maximilian.hornauer@study.thws.de),
 *          Usama Ali (usama.ali@study.thws.de)
 * @brief Header to task_board_tasks.cpp
 * @version 0.1
 * @date 2023-04-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ur_ros_driver/SetCartTarget.h>
#include <ur_ros_driver/SetTrajectory.h>
#include <ur_ros_driver/SetJointTarget.h>
#include <ur_ros_driver/SetForceTarget.h>
#include <ur_ros_driver/SetContactTarget.h>
#include <ur_ros_driver/SetGripper.h>
#include <ur_ros_driver/StartJog.h>

//#include <ur_ros_driver/ros_color_stream.hpp>

#include <ur_ros_driver/PathEntry.h>
#include <ur_ros_driver/JogControl.h>
#include <ur_ros_driver/GripperInfo.h>
#include <ur_ros_driver/Log.h>
#include <ur_ros_driver/GetTimeStamp.h>
#include <robothon2023/GetTriangles.h>
#include <robothon2023/GetBoardLocation.h>
#include <robothon2023/GetFinished.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <ur_ros_driver/ros_color_stream.hpp>

typedef boost::array<double, 6> array6d;
typedef boost::array<long int, 6> array6i;

class task_board_tasks
{
private:
    tf2_ros::Buffer *tfBuffer_;
    tf2_ros::TransformListener *tfListener_;

    ur_ros_driver::SetTrajectory trajectory;

    ros::ServiceClient cart_move_;
    ros::ServiceClient path_move_;
    ros::ServiceClient joint_move_;
    ros::ServiceClient force_move_;
    ros::ServiceClient contact_move_;
    ros::ServiceClient gripper_move_;
    ros::ServiceClient move_trajectory_;
    ros::ServiceClient triangle_detection_;
    ros::ServiceClient taskboard_detection_;
    ros::ServiceClient finish_detection_;
    ros::ServiceClient logging_;
    ros::ServiceClient robot_time_;

    ros::Subscriber gripper_info_sub_;
    ros::Subscriber tcp_position_sub_;
    ros::Subscriber wrench_sub_;

    ur_ros_driver::GripperInfo gripper_info_;
    geometry_msgs::TransformStamped tcp_position_;
    geometry_msgs::WrenchStamped wrench_;

    double vel_multi_ = 1;
    double acc_multi_ = 1;

    double travel_vel_ = 3;
    double travel_acc_ = 3;

    double joint_vel_ = 3;
    double joint_acc_ = 3;

    double wind_vel_ = 2.5;
    double wind_acc_ = 2;

    double blend_min_ = 0.0002;

    ur_ros_driver::SetCartTarget cart_target(std::string target, int mode = 2, double vel = 1, double acc = 1);
    ur_ros_driver::SetForceTarget force_target(bool IO, array6d free_axis = {0,0,1,0,0,0},array6d wrench = {0,0,-10.0,0,0,0}, double vel=3);
    ur_ros_driver::SetJointTarget joint_target(array6d target, double vel = 0.5, double acc = 0.5);
    ur_ros_driver::SetContactTarget contact_target(array6d speed = {0,0,-0.1,0,0,0}, double acc = 0.5);
    ur_ros_driver::SetGripper gripper_open(double position = 100, double speed = 100, double force = 10);
    ur_ros_driver::SetGripper gripper_close(double position = 0, double speed = 100, double force = 100);

    void add_cart_Path_Entry(std::string target, int move_type = 1, double blend = 0, double vel = 0.5, double acc = 0.5);
    void add_joint_Path_Entry(array6d target, int move_type = 0, double blend = 0, double vel = 0.5, double acc = 0.5);

    bool call_cart_target(ur_ros_driver::SetCartTarget& srv);
    bool call_force_target(ur_ros_driver::SetForceTarget& srv);
    bool call_joint_target(ur_ros_driver::SetJointTarget& srv);
    bool call_contact_target(ur_ros_driver::SetContactTarget& srv);
    bool call_gripper(ur_ros_driver::SetGripper& srv);
    bool call_triangles(robothon2023::GetTriangles& srv);
    bool call_jog_start(bool IO);
    bool call_finish_detection();
    


    //bool service_call(ros::ServiceClient &client, auto &srv);

public:
    int config = 2 ;
    
    task_board_tasks(ros::NodeHandle& n);
    ~task_board_tasks();
    
    bool call_trajectory();
    bool call_board_detection();
    bool call_log(bool IO, std::string filename);
    bool call_robot_time();

    void gripper_info_callback(ur_ros_driver::GripperInfo msg);
    void tcp_positon_callback(geometry_msgs::TransformStamped msg);
    void wrench_callback(geometry_msgs::WrenchStamped msg);

    bool press_button(std::string color);

    bool move_plug(std::string from, std::string to);

    bool center_slider();
    bool move_slider();
    bool slider_fallback();

    bool grab_probe();
    bool open_door();
    bool measure();
    bool return_probe();

    bool get_hook(bool bring = false);
    bool get_hook_new(bool bring = false);
    bool hook_cable();
    bool wind_cable();
    bool press_button_hook();

    bool check_finish();

    bool home(bool fast = false);
    bool spiral_force(double start, double stop, int step_c , double step_f, double z_limit,bool dir = true);
    double pose_distance(geometry_msgs::Transform target);
    bool function_test();
    bool calculate_config();
};
