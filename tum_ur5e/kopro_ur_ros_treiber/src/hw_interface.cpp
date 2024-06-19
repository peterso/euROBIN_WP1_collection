/**
 * @file hw_interface.cpp
 * @author Adrian Mueller (adrian.mueller@study.thws.de)
 * @brief 
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ur_ros_driver/hw_interface.hpp>

URHardwareInterface::URHardwareInterface()
    : joint_positions_( { 0, 0, 0, 0, 0, 0 } )
    , joint_velocities_( { 0, 0, 0, 0, 0, 0 } )
    , joint_efforts_( { 0, 0, 0, 0, 0, 0 } )
{  

}

bool URHardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
    this->nh_ = robot_hw_nh;
    this->joint_names_ = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    
    ROS_GREEN_STREAM("Init Paramter");
    this->initParameter(robot_hw_nh);
    ROS_GREEN_STREAM("Init Interfaces");
    this->initInterfaces(root_nh,robot_hw_nh);
    ROS_GREEN_STREAM("Init Controller");
    this->initROSInterfaces(robot_hw_nh);
    ROS_GREEN_STREAM("Init Services");
    this->initServices(robot_hw_nh);
    ROS_GREEN_STREAM("Init Topics");
    this->initTopics(robot_hw_nh);
    
    rtde_control_->zeroFtSensor();

    return true;
}

bool URHardwareInterface::initParameter(ros::NodeHandle& robot_hw_nh)
{
    if (!robot_hw_nh.getParam("robot_ip", robot_ip_))
    {
        ROS_ERROR_STREAM("Required parameter robot_ip not given. hw");
        return false;
    }
    return true;
}

void URHardwareInterface::initInterfaces(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
    
    this->dashboard_ = new DashboardClient();
    this->gripper_ = new RobotiqGripperInterface();
    this->dashboard_->init(root_nh,robot_hw_nh);

    this->rtde_receive_ = new ur_rtde::RTDEReceiveInterface(robot_ip_);
    this->start_robot();

    ROS_GREEN_STREAM("Robot is running");

    while(!dashboard_->isInRemoteControl())
    {
            ROS_WARN_STREAM_THROTTLE(5,"Please set Robot remote control");
    }

    this->gripper_->init(root_nh,robot_hw_nh);
    this->rtde_control_ = new ur_rtde::RTDEControlInterface(robot_ip_);
}

void URHardwareInterface::initROSInterfaces(ros::NodeHandle& robot_hw_nh)
{
    for (int i = 0; i < joint_positions_.size(); ++i)
    {
        joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_positions_[i],
                                                                          &joint_velocities_[i], &joint_efforts_[i]));
    }

    this->registerInterface(&joint_state_interface_);
}

void URHardwareInterface::initServices(ros::NodeHandle& robot_hw_nh)
{
    set_cart_target_srv_ = robot_hw_nh.advertiseService("/set_cart_target", &URHardwareInterface::set_cart_target, this);
    set_rel_cart_target_srv_ = robot_hw_nh.advertiseService("/set_rel_cart_target", &URHardwareInterface::set_rel_cart_target, this);
    set_joint_target_srv_ = robot_hw_nh.advertiseService("/set_joint_target", &URHardwareInterface::set_joint_target, this);
    set_rel_joint_target_srv_ = robot_hw_nh.advertiseService("/set_rel_joint_target", &URHardwareInterface::set_rel_joint_target, this);
    set_trajectory_srv_ = robot_hw_nh.advertiseService("/set_trajectory", &URHardwareInterface::set_trajectory, this);
    set_freedive_srv_ = robot_hw_nh.advertiseService("/set_freedive", &URHardwareInterface::set_freedive, this);
    set_force_target_srv_ = robot_hw_nh.advertiseService("/set_force_mode", &URHardwareInterface::set_force_target, this);
    set_contact_target_srv_ = robot_hw_nh.advertiseService("/set_contact_target", &URHardwareInterface::set_contact_target, this);
    start_jog_srv_ = robot_hw_nh.advertiseService("/start_jog", &URHardwareInterface::start_jog, this);
    zero_ftsensor_srv_ = robot_hw_nh.advertiseService("/zero_ftsensor", &URHardwareInterface::zero_ftsensor, this);
    log_srv_ = robot_hw_nh.advertiseService("/logging", &URHardwareInterface::log, this);
    set_payload_srv_ = robot_hw_nh.advertiseService("/set_payload", &URHardwareInterface::set_payload, this);
    get_timestamp_srv_ = robot_hw_nh.advertiseService("/get_robot_timestamp", &URHardwareInterface::get_timestamp, this);
}

void URHardwareInterface::initTopics(ros::NodeHandle& robot_hw_nh)
{
    
    tcp_pose_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::TransformStamped>(robot_hw_nh, "/tcp_pose", 100));
    wrench_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(robot_hw_nh, "/wrench", 100));
}

bool URHardwareInterface::start_robot()
{
    ROS_GREEN_STREAM("Try to start robot");
    if(rtde_receive_->getRobotMode()==3)
    {
        ROS_GREEN_STREAM("power on");
        while(rtde_receive_->getRobotMode()!=5)
        {
            dashboard_->powerOn();
            ros::Duration(0.5).sleep();
        }
    }

    if(rtde_receive_->getRobotMode()==5)
    {
        ROS_GREEN_STREAM("brake release");

        while(rtde_receive_->getRobotMode()!=7)
        {
            dashboard_->brakeRelease();
            ros::Duration(0.5).sleep();
        }
    }
    return true;
}

void URHardwareInterface::check_state()
{
    if(!rtde_receive_->isConnected())
    {
        rtde_receive_->reconnect();
    }
    int safetyStatus = rtde_receive_->getSafetyMode();
    switch (safetyStatus)
    {
    case ur_rtde::RTDEReceiveInterface::IS_NORMAL_MODE:
        break;
    case ur_rtde::RTDEReceiveInterface::IS_RECOVERY_MODE:
        ROS_WARN_STREAM_THROTTLE(5,"SAFETY STATUS: PROTECTIVE_STOPP");
        break;
    case ur_rtde::RTDEReceiveInterface::IS_EMERGENCY_STOPPED:
        ROS_WARN_STREAM_THROTTLE(5,"SAFETY STATUS: EMERGENCY_STOPP");
        break;
    case ur_rtde::RTDEReceiveInterface::IS_VIOLATION:
        ROS_WARN_STREAM_THROTTLE(5,"SAFETY STATUS: VIOLATION");
        break;
    case ur_rtde::RTDEReceiveInterface::IS_FAULT:
        ROS_WARN_STREAM_THROTTLE(5,"SAFETY STATUS: FAULT");
        break;
    case ur_rtde::RTDEReceiveInterface::IS_STOPPED_DUE_TO_SAFETY:
        ROS_WARN_STREAM_THROTTLE(5,"SAFETY STATUS: STOPPED_DUE_TO_SAFETY");
        break;
    default:
        break;
    } 

}

void URHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
    this->read_connection_state();
    this->read_tcp_pose();
    this->read_joint_values();
    this->read_wrench();
}

void URHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
    this->publish_tcp_pose();
    this->publish_wrench();
}

bool URHardwareInterface::set_cart_target(ur_ros_driver::SetCartTargetRequest& req,
                                          ur_ros_driver::SetCartTargetResponse& res)
{
    std::vector<double> pose;
    transform_to_angelaxis(req.cartesian_goal,pose);
    bool success,error;

    try{
        if(!rtde_control_->isPoseWithinSafetyLimits(pose))
        {
            ROS_WARN("Target is out of range or savety limits");
            res.success = false;
            return false;
        }

        switch (req.mode)
        {
        case 1:
            success = rtde_control_->moveL(pose,req.speed,req.acceleration,req.asynchronous);
            break;
        case 2:
            success = rtde_control_->moveJ_IK(pose,req.speed,req.acceleration,req.asynchronous);
            break;
        default:
            error = true;
            success = false;
            break;
        }

        if(req.asynchronous && !error)
        {res.success = true;}
        else{res.success = success;}
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
        res.success = false;
        return true;
    }

    return true;
}

bool URHardwareInterface::set_rel_cart_target(ur_ros_driver::SetRelCartTargetRequest& req,
                                              ur_ros_driver::SetRelCartTargetResponse& res)
{
    std::vector<double> pose;
    geometry_msgs::Transform trans = this->tcp_pose_;
    trans.translation.x =+ req.rel_goal[0];
    trans.translation.y =+ req.rel_goal[1];
    trans.translation.z =+ req.rel_goal[2];
    //TODO: Add relative rotation
    Eigen::Quaterniond quat_rel = Eigen::AngleAxisd(req.rel_goal[3], Eigen::Vector3d::UnitX())
                                * Eigen::AngleAxisd(req.rel_goal[4], Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(req.rel_goal[5], Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond quat;
    tf::quaternionMsgToEigen(trans.rotation,quat);
    quat = quat*quat_rel;
    tf::quaternionEigenToMsg(quat,trans.rotation);
    transform_to_angelaxis(trans,pose);

    bool success,error;

    try{
        if(!rtde_control_->isPoseWithinSafetyLimits(pose))
        {
            ROS_WARN("Target is out of range or savety limits");
            res.success = false;
            return false;
        }

        switch (req.mode)
        {
        case 1:
            success = rtde_control_->moveL(pose,req.speed,req.acceleration,req.asynchronous);
            break;
        case 2:
            success = rtde_control_->moveJ_IK(pose,req.speed,req.acceleration,req.asynchronous);
            break;
        default:
            error = true;
            success = false;
            break;
        }

        if(req.asynchronous && !error)
        {res.success = true;}
        else{res.success = success;}
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
        res.success = false;
        return true;
    }

    return true;
}

bool URHardwareInterface::set_joint_target(ur_ros_driver::SetJointTargetRequest& req,
                                           ur_ros_driver::SetJointTargetResponse& res)
{
    try{
        if(!rtde_control_->isJointsWithinSafetyLimits(std::vector<double>(std::begin(req.joint_goal),std::end(req.joint_goal))))
        {
            ROS_WARN("Target is out of joint or savety limits");
            res.success = false;
            return false;
        }

        res.success = rtde_control_->moveJ(
            std::vector<double>(std::begin(req.joint_goal),std::end(req.joint_goal)),
            req.speed,req.acceleration,req.asynchronous);
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
        res.success = false;
        return true;
    }
    return true;
}

bool URHardwareInterface::set_rel_joint_target(ur_ros_driver::SetJointTargetRequest& req,
                                           ur_ros_driver::SetJointTargetResponse& res)
{
    std::vector<double> joint_goal(joint_positions_.begin(),joint_positions_.end());
    std::vector<double> rel_goal = std::vector<double>(std::begin(req.joint_goal),std::end(req.joint_goal));
    for(int i=0;i<6;i++)
    {
        joint_goal[i]+=rel_goal[i];
    }
    
    try{
        if(!rtde_control_->isJointsWithinSafetyLimits(joint_goal))
        {
            ROS_WARN("Target is out of joint or savety limits");
            res.success = false;
            return false;
        }

        res.success = rtde_control_->moveJ(joint_goal,req.speed,req.acceleration,req.asynchronous);
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
        res.success = false;
        return true;
    }
    return true;
}

bool URHardwareInterface::set_trajectory(ur_ros_driver::SetTrajectoryRequest& req,
                                              ur_ros_driver::SetTrajectoryResponse& res)
{
    ur_rtde::Path path;
    try{
        for(int i = 0; i < req.path.size(); i++)
        {
            std::vector<double> parameter;

            geometry_msgs::Quaternion temp;
            if(req.path[i].cartesian_goal.rotation != temp)
            {
                transform_to_angelaxis(req.path[i].cartesian_goal,parameter);
            }
            else
            {
                parameter = std::vector<double>(std::begin(req.path[i].joint_goal),std::end(req.path[i].joint_goal));
                
            }

            parameter.push_back(req.path[i].velocity);
            parameter.push_back(req.path[i].acceleration);
            parameter.push_back(req.path[i].blend);
        
            ur_rtde::PathEntry entry(ur_rtde::PathEntry::eMoveType(req.path[i].move_type),
                                    ur_rtde::PathEntry::ePositionType(req.path[i].position_type),
                                    parameter);
                
            path.addEntry(entry);
        }

        res.success = rtde_control_->movePath(path,req.asynchronous);
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
        res.success = false;
        return true;
    }
    
    return true;
}

bool URHardwareInterface::set_force_target(ur_ros_driver::SetForceTargetRequest& req,
                                           ur_ros_driver::SetForceTargetResponse& res)
{
    static bool force_mode_activ_ = false;
    try{
        if(req.IO)
        {
            if(!force_mode_activ_)
            {
                while(!rtde_control_->isSteady())
                {
                    ros::Duration(0.01).sleep();
                    continue;
                }
                rtde_control_->zeroFtSensor();
                ros::Duration(0.1).sleep();
                std::cout << "Force Mode Activated" << std::endl;
            }

            std::vector<double> task_frame;
            geometry_msgs::Quaternion temp;
            if(req.frame.rotation != temp)
            {
                transform_to_angelaxis(req.frame,task_frame);
            }
            else
            {
                task_frame = {0,0,0,0,0,0};
            }

            res.success = rtde_control_->forceMode(task_frame,
                std::vector<int>(std::begin(req.selection_vector),std::end(req.selection_vector)),
                std::vector<double>(std::begin(req.wrench),std::end(req.wrench)),
                req.type,
                std::vector<double>(std::begin(req.limits),std::end(req.limits)));
            force_mode_activ_ = res.success;
        }
        else
        {
            if(force_mode_activ_)
            {
                res.success = rtde_control_->forceModeStop();  
                force_mode_activ_ = !res.success;
                std::cout << "Force Mode Deactivated" << std::endl;
            }
            else
            {
                res.success = true;
            }
        }
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
        res.success = false;
        return true;
    }
 
    return true;
}

bool URHardwareInterface::set_contact_target(ur_ros_driver::SetContactTargetRequest& req,
                                           ur_ros_driver::SetContactTargetResponse& res)
{
    try{
        res.success = rtde_control_->moveUntilContact(
            std::vector<double>(std::begin(req.xd),std::end(req.xd)),
            std::vector<double>(std::begin(req.direction),std::end(req.direction)),
            req.acceleration);
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
        res.success = false;
        return true;
    }
    return true;
}

bool URHardwareInterface::set_freedive(ur_ros_driver::SetFreedriveRequest& req,
                                       ur_ros_driver::SetFreedriveResponse& res)
{
    try{
        if(req.IO)
        {
            std::vector<double> frame;
            switch(req.feature) {
                case 0:
                    frame = {0,0,0,0,0,0};
                    break;
                case 1:
                    frame = rtde_receive_->getActualTCPPose();
                    break;
                case 2:
                    transform_to_angelaxis(req.custom_frame ,frame);
                    break;
            }
            res.success = rtde_control_->freedriveMode(
                std::vector<int>(std::begin(req.free_axes),std::end(req.free_axes)),
                frame);
            if(res.success)
                std::cout << "Freedrive Mode Activated" << std::endl;
        }
        else
        {
            res.success = rtde_control_->endFreedriveMode();

            if(res.success)
                std::cout << "Freedrive Mode Deactivated" << std::endl;
        }

        res.FreedriveStatus = rtde_control_->getFreedriveStatus();
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
        res.success = false;
        return true;
    }

    return true;
}

bool URHardwareInterface::start_jog(ur_ros_driver::StartJogRequest& req, 
                                    ur_ros_driver::StartJogResponse& res)
{
    try{
    if(req.IO)
    {
        jog_conroll_sub_ = nh_.subscribe("/jog_control",1000,&URHardwareInterface::jog_control_callback,this);
    }
    else{
        jog_conroll_sub_.shutdown();
        rtde_control_->jogStop();
    }
    res.success = true;
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
        res.success = false;
        return true;
    }
    return true;
}

bool URHardwareInterface::zero_ftsensor(std_srvs::TriggerRequest& req, 
                                        std_srvs::TriggerResponse& res)
{
    try{
        while(!rtde_control_->isSteady())
        {
            ros::Duration(0.01).sleep();
            continue;
        }
        rtde_control_->zeroFtSensor();
        return true;
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
    }
    return false;
}

bool URHardwareInterface::log(ur_ros_driver::LogRequest& req, 
                              ur_ros_driver::LogResponse& res)
{
    try{
        if(req.IO)
        {     
            if (req.file_name.empty())
            {
                ROS_WARN_STREAM("log filename is missing");
                res.success = false;
                return false;
            }
            std::vector<std::string> record_variables = {"timestamp", "actual_q", "actual_TCP_pose"};
            // std::vector<std::string> record_variables = req.parameter;
            res.success = rtde_receive_->startFileRecording(req.file_name + ".csv");
        }
        else
        {
            res.success = rtde_receive_->stopFileRecording();
        }
        return true;
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
    }
    return false;
}

bool URHardwareInterface::set_payload(ur_ros_driver::SetPayloadRequest& req, 
                                      ur_ros_driver::SetPayloadResponse& res)
{
    try{
        res.success = rtde_control_->setPayload(req.mass,
            std::vector<double>(std::begin(req.cog),std::end(req.cog)));
        return true;
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
    }
    res.success = false;
    return false;
}

bool URHardwareInterface::get_timestamp(ur_ros_driver::GetTimeStampRequest& req, ur_ros_driver::GetTimeStampResponse& res)
{
    try{
        res.time = rtde_receive_->getTimestamp();
        return true;
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
    }
    res.time = 0;
    return false;
}

void URHardwareInterface::jog_control_callback(const ur_ros_driver::JogControl& msg)
{   
    try{
        if(msg.stamp + ros::Duration(3) < ros::Time::now())
        {
            rtde_control_->jogStop();
            return;
        }

        if(msg.feature == 1)
        {
            rtde_control_->jogStart(
                std::vector<double>(std::begin(msg.speeds),std::end(msg.speeds)),
                ur_rtde::RTDEControlInterface::FEATURE_BASE);
        }
        else if(msg.feature == 2)
        {
            rtde_control_->jogStart(
                std::vector<double>(std::begin(msg.speeds),std::end(msg.speeds)),
                ur_rtde::RTDEControlInterface::FEATURE_TOOL);
        }
        else if(msg.feature == 3)
        {
            rtde_control_->jogStart(
                std::vector<double>(std::begin(msg.speeds),std::end(msg.speeds)),
                ur_rtde::RTDEControlInterface::FEATURE_CUSTOM,
                std::vector<double>(std::begin(msg.custom_frame),std::end(msg.custom_frame)));
        }
        else
        {
            rtde_control_->jogStop();
        }
    }
    catch(std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
    }
}

void URHardwareInterface::publish_tcp_pose()
{
    if (tcp_pose_pub_)
    {
        if (tcp_pose_pub_->trylock())
        {
            tcp_pose_pub_->msg_.transform = tcp_pose_;
            tcp_pose_pub_->msg_.header.stamp = ros::Time::now();
            tcp_pose_pub_->unlockAndPublish();
        }
    }
}

void URHardwareInterface::publish_wrench()
{
    if (wrench_pub_)
    {
        if (wrench_pub_->trylock())
        {
            wrench_pub_->msg_.wrench = wrench_;
            wrench_pub_->msg_.header.stamp = ros::Time::now();
            wrench_pub_->unlockAndPublish();
        }
    }
}

void URHardwareInterface::read_connection_state()
{
    while(!rtde_receive_->isConnected())
    {
        try{
            ROS_YELLOW_STREAM_THROTTLE(5,"Try to restart recevie Interface");
            rtde_receive_->reconnect();
        }
        catch(std::exception& e) {
            ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
        }
    }

    while(!rtde_control_->isConnected())
    {
        try{
            ROS_YELLOW_STREAM_THROTTLE(5,"Try to restart control Interface");
            rtde_control_->reconnect();
        }
        catch(std::exception& e) {
            ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
        }
    }

    while(rtde_receive_->getRobotMode()!=7)
    {
        ROS_YELLOW_STREAM_THROTTLE(5,"Try to reupload control Script");
        rtde_control_->reuploadScript();
    }
}

void URHardwareInterface::read_tcp_pose()
{
    Eigen::Matrix<double,6,1> pose_vec(rtde_receive_->getActualTCPPose().data());
    Eigen::Vector3d rotVec = pose_vec.block<3,1>(3,0); 
    Eigen::Matrix3d rotMat = Eigen::AngleAxisd(rotVec.norm(), rotVec.normalized()).matrix();

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3,3>(0,0) = rotMat;
    pose.block<3,1>(0,3) = pose_vec.block<3,1>(0,0);

    Eigen::Isometry3d iso(pose);
    tf::transformEigenToMsg(iso,tcp_pose_);
}

void URHardwareInterface::read_joint_values()
{
    std::copy_n(rtde_receive_->getActualQ().begin(), 6, joint_positions_.begin());
    std::copy_n(rtde_receive_->getActualQd().begin(), 6, joint_velocities_.begin());
}

void URHardwareInterface::read_wrench()
{
    Eigen::Matrix<double,6,1> wrench(rtde_receive_->getActualTCPForce().data());

    tf::vectorEigenToMsg(wrench.block<3,1>(0,0),wrench_.force);
    tf::vectorEigenToMsg(wrench.block<3,1>(3,0),wrench_.torque);
}

void URHardwareInterface::transform_to_angelaxis(geometry_msgs::Transform& transform, std::vector<double>& angleaxis)
{
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(transform.rotation,q);
    Eigen::AngleAxisd angleAxis(q);
    Eigen::Vector3d angles = angleAxis.axis() * angleAxis.angle();

    angleaxis.clear();
    
    angleaxis = {transform.translation.x,
                    transform.translation.y,
                    transform.translation.z};

    angleaxis.insert(angleaxis.end(),angles.data(),angles.data() + angles.size());
}

std::unique_ptr<URHardwareInterface> hw_interface;

// void thread(URHardwareInterface& hw_interface)
// {
//     while(ros::ok())
//     {
//         //rtde_receive_->getRobotMode()         int    
//         //rtde_receive_->getSafetyMode()        int
//         //rtde_receive_->getSafetyStatusBits()  int     Bits 0-10:
//         //rtde_receive_->isEmergencyStopped()   bool
//         //rtde_receive_->isProtectiveStopped()  bool
//         //rtde_receive_->isConnected()          bool
//         //rtde_control_->isConnected()          bool
//         //rtde_receive_->reconnect()            bool
//         //rtde_control_->reconnect()            bool

//         //rtde_receive_->  getRobotStatus()     int     Bits 0-3: Is power on | Is program running | Is teach button pressed | Is power button pressed
//     }
// }

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ur_hardware_interface");
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    ros::Rate rate(100);

    ros::Time timestamp;
    ros::Duration period;
    auto stopwatch_last = std::chrono::steady_clock::now();
    auto stopwatch_now = stopwatch_last;

    hw_interface.reset(new URHardwareInterface);
    ROS_GREEN_STREAM("Driver is starting");
    if (!hw_interface->init(nh, nh_priv))
    {
        ROS_ERROR_STREAM("Could not correctly initialize robot. Exiting");
        exit(1);
    }
    ROS_GREEN_STREAM("Driver is now ready");

    controller_manager::ControllerManager cm(hw_interface.get(), nh);

    while (ros::ok())
    {
        try{
            // Get current time and elapsed time since last read
            timestamp = ros::Time::now();
            stopwatch_now = std::chrono::steady_clock::now();
            period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
            stopwatch_last = stopwatch_now;

            // Receive current state from robot
            hw_interface->check_state();
            hw_interface->read(timestamp, period);
            cm.update(timestamp, period);
            hw_interface->write(timestamp, period);
        }
        catch(std::exception& e) {
            ROS_ERROR_STREAM_THROTTLE(5,"error: " << e.what());
            return 1;
        }
        catch(...) {
            ROS_ERROR_STREAM_THROTTLE(5,"Exception of unknown type!");
        }
        rate.sleep();


    }

    return 0;
}