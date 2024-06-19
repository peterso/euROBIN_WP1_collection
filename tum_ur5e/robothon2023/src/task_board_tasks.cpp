/**
 * @file task_board_tasks.cpp
 * @authors Adrian Müller (adrian.mueller@study.thws.de), 
 *          Maximilian Hornauer (maximilian.hornauer@study.thws.de),
 *          Usama Ali (usama.ali@study.thws.de)
 * @brief Program to execute tasks
 * @version 0.1
 * @date 2023-04-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include <robothon2023/task_board_tasks.hpp>

task_board_tasks::task_board_tasks(ros::NodeHandle& n)
{
    tfBuffer_ = new tf2_ros::Buffer();
    tfListener_ = new tf2_ros::TransformListener(*tfBuffer_);

    trajectory.request.asynchronous = false;

    cart_move_ = n.serviceClient<ur_ros_driver::SetCartTarget>("/set_cart_target");
    joint_move_ = n.serviceClient<ur_ros_driver::SetJointTarget>("/set_joint_target");
    force_move_ = n.serviceClient<ur_ros_driver::SetForceTarget>("/set_force_mode");
    contact_move_ = n.serviceClient<ur_ros_driver::SetContactTarget>("/set_contact_target");
    gripper_move_ = n.serviceClient<ur_ros_driver::SetGripper>("/set_gripper");
    move_trajectory_ = n.serviceClient<ur_ros_driver::SetTrajectory>("/set_trajectory");
    triangle_detection_ = n.serviceClient<robothon2023::GetTriangles>("/triangle_detection");
    taskboard_detection_ = n.serviceClient<robothon2023::GetBoardLocation>("/board_detection");
    finish_detection_ = n.serviceClient<robothon2023::GetFinished>("/finished_detection");
    logging_ = n.serviceClient<ur_ros_driver::Log>("/logging");
    robot_time_ = n.serviceClient<ur_ros_driver::GetTimeStamp>("/get_robot_timestamp");

    gripper_info_sub_ = n.subscribe("/gripper_infos",100,&task_board_tasks::gripper_info_callback,this);
    tcp_position_sub_ = n.subscribe("/tcp_pose",100,&task_board_tasks::tcp_positon_callback,this);
    wrench_sub_ = n.subscribe("/wrench",100,&task_board_tasks::wrench_callback,this);
}

task_board_tasks::~task_board_tasks()
{
}

void task_board_tasks::gripper_info_callback(ur_ros_driver::GripperInfo msg)
{
    gripper_info_ = msg;
}

void task_board_tasks::tcp_positon_callback(geometry_msgs::TransformStamped msg)
{
    tcp_position_ = msg;
}

void task_board_tasks::wrench_callback(geometry_msgs::WrenchStamped msg)
{
    wrench_ = msg;
}

ur_ros_driver::SetCartTarget task_board_tasks::cart_target(std::string target, int mode, double vel, double acc)
{
    ur_ros_driver::SetCartTarget srv;
    srv.request.cartesian_goal = tfBuffer_->lookupTransform("base",target,ros::Time(0)).transform;
    srv.request.mode = mode;
    srv.request.speed = vel * vel_multi_;
    srv.request.acceleration = acc * acc_multi_;
    srv.request.asynchronous = false;
    return srv;
}

ur_ros_driver::SetForceTarget task_board_tasks::force_target(bool IO, array6d free_axis ,array6d wrench, double vel)
{
    vel = vel*vel_multi_;
    ur_ros_driver::SetForceTarget srv;
    srv.request.IO = IO;
    srv.request.wrench = wrench;
    srv.request.limits = {vel,vel,vel,vel,vel,vel};
    srv.request.selection_vector = free_axis;
    srv.request.type = 2;
    return srv;
}

ur_ros_driver::SetJointTarget task_board_tasks::joint_target(array6d target, double vel, double acc)
{
    ur_ros_driver::SetJointTarget srv;
    srv.request.joint_goal = target;
    srv.request.speed = vel * vel_multi_;
    srv.request.acceleration = acc * acc_multi_;
    srv.request.asynchronous = false;
    return srv;
}

ur_ros_driver::SetContactTarget task_board_tasks::contact_target(array6d speed, double acc)
{
    ur_ros_driver::SetContactTarget srv;
    srv.request.xd = speed;
    srv.request.direction = {0,0,0,0,0,0};
    srv.request.acceleration = acc * acc_multi_;
    return srv;
}

ur_ros_driver::SetGripper task_board_tasks::gripper_open(double position, double speed, double force)
{
    ur_ros_driver::SetGripper srv;
    srv.request.position = position;
    srv.request.force = force;
    srv.request.speed = speed;
    srv.request.asynchronous = false;
    return srv;
}

ur_ros_driver::SetGripper task_board_tasks::gripper_close(double position, double speed, double force)
{
    ur_ros_driver::SetGripper srv;
    srv.request.position = position;
    srv.request.position_unit = 0;
    srv.request.force = force;
    srv.request.speed = speed;
    srv.request.asynchronous = false;
    return srv;
}

void task_board_tasks::add_cart_Path_Entry(std::string target, int move_type, double blend, double vel, double acc)
{
    ur_ros_driver::PathEntry entry;
    entry.cartesian_goal = tfBuffer_->lookupTransform("base",target,ros::Time(0)).transform;
    entry.velocity = vel* vel_multi_;
    entry.acceleration = acc* acc_multi_;
    entry.blend = blend;
    entry.position_type = 0;
    entry.move_type = move_type;

    trajectory.request.path.push_back(entry);
}

void task_board_tasks::add_joint_Path_Entry(array6d target, int move_type, double blend, double vel, double acc)
{
    ur_ros_driver::PathEntry entry;
    entry.joint_goal = target;
    entry.velocity = vel* vel_multi_;
    entry.acceleration = acc* acc_multi_;
    entry.blend = blend;
    entry.position_type = 1;
    entry.move_type = move_type;

    trajectory.request.path.push_back(entry);
}

bool task_board_tasks::call_cart_target(ur_ros_driver::SetCartTarget& srv)
{
    if(!cart_move_.call(srv))
    {
        // throw("Failed to call service /cart_move");
        return false;
    }
    if(!srv.response.success)
    {
    //    throw("Failed to move to cart_target");
        return false;
    }
    return true;
}

bool task_board_tasks::call_force_target(ur_ros_driver::SetForceTarget& srv)
{
    if(!force_move_.call(srv))
    {
        // throw("Failed to call service /force_move");
        return false;
    }
    if(!srv.response.success)
    {
        // throw("Failed to move to force_target");
        return false;
    }
    return true;
}

bool task_board_tasks::call_joint_target(ur_ros_driver::SetJointTarget& srv)
{
    if(!joint_move_.call(srv))
    {
        // throw("Failed to call service /joint_move");
        return false;
    }
    if(!srv.response.success)
    {
        // throw("Failed to move to joint_move");
        return false;
    }
    return true;
}

bool task_board_tasks::call_contact_target(ur_ros_driver::SetContactTarget& srv)
{
    if(!contact_move_.call(srv))
    {
        // throw("Failed to call service /contact_move");
        return false;
    }
    if(!srv.response.success)
    {
        // throw("Failed to move to contact_move");
        return false;
    }
    return true;
}

bool task_board_tasks::call_gripper(ur_ros_driver::SetGripper& srv)
{
    if(!gripper_move_.call(srv))
    {
        // throw("Failed to call service /gripper_move");
        return false;
    }
    if(!srv.response.success)
    {
        // throw("Failed to move to gripper_move");
        return false;
    }
    return true;
}


bool task_board_tasks::call_triangles(robothon2023::GetTriangles& srv)
{
    if(!triangle_detection_.call(srv))
    {   
        // throw("Failed to call service /triangles");
        srv.response.status=-1;
        return true;
    }
    //ROS_INFO_STREAM("Triangle delta:  " << srv.response.delta << " Status: " << srv.response.status );
    return true;
}

bool task_board_tasks::call_board_detection()
{
    robothon2023::GetBoardLocation srv;
    if(!taskboard_detection_.call(srv))
    {
        // throw("Failed to call service /taskboard_detection");
        return false;
    }
    if(srv.response.success == false)
    {
        // throw("Failed to detect taskboard");
        return false;
    }

    config = srv.response.config;
    //std::cout << "Config: " << config << std::endl;
    return true;
}

bool task_board_tasks::call_trajectory()
{   
    if(!move_trajectory_.call(trajectory))
    {
        trajectory.request.path.clear();
        // throw("Failed to call service /set_trajectory");
        return false;
    }
    if(!trajectory.response.success)
    {
        trajectory.request.path.clear();
        // throw("Failed to move to trajectory");
        return false;
    }
    trajectory.request.path.clear();
    return true;
}

bool task_board_tasks::call_finish_detection()
{
    robothon2023::GetFinished srv;
    if(!finish_detection_.call(srv))
    {
        // throw("Failed to call service /taskboard_detection");
        return false;
    }
    if(srv.response.success == false)
    {
        // throw("Failed to detect taskboard");
        return false;
    }
    return true;
}

bool task_board_tasks::call_log(bool IO, std::string filename)
{
    ur_ros_driver::Log srv;
    srv.request.IO = IO;
    
    srv.request.file_name = "/home/robothon/robothon_logs/"+filename; //NOTE: Change the log path if needed!
    if(!logging_.call(srv))
    {
        // throw("Failed to call service /taskboard_detection");
        return false;
    }
    if(srv.response.success == false)
    {
        // throw("Failed to detect taskboard");
        return false;
    }
    if(IO)
        ROS_YELLOW_STREAM("started Logging");
    else
        ROS_YELLOW_STREAM("stopped Logging");
    return true;
}

bool task_board_tasks::call_robot_time()
{
    call_trajectory();
    ur_ros_driver::GetTimeStamp srv;
    if(!robot_time_.call(srv))
    {
        // throw("Failed to call service /taskboard_detection");
        return false;
    }
    ROS_MAGENTA_STREAM("actual Robot time: " << srv.response.time);
    return true;
}

bool task_board_tasks::press_button(std::string color)
{

    ur_ros_driver::SetForceTarget btn = force_target(true,{0,0,1,0,0,0},{0,0,-20,0,0,0});
    ur_ros_driver::SetGripper g_close = gripper_close();
    g_close.request.asynchronous = true;

    if(color== "red")
        add_cart_Path_Entry("btn_red_helper", 1,blend_min_,travel_vel_,travel_acc_);

    if(color == "blue" || color == "red")
    {
        call_trajectory();
        call_gripper(g_close);

        add_cart_Path_Entry("btn_"+color+"_app", 1,0.005,travel_vel_,travel_acc_);
        // add_cart_Path_Entry("btn_"+color+"_app", 1);
        if(color == "blue")
        {
            add_cart_Path_Entry("btn_"+color+"_app_short", 1,blend_min_,travel_vel_,travel_acc_);
        }
        call_trajectory();

        call_force_target(btn);
        while(wrench_.wrench.force.z < 10 && ros::ok())
        {
            ros::Duration(0.5).sleep();
            continue;
        }
        btn.request.IO = false;
        call_force_target(btn);

        add_cart_Path_Entry("btn_"+color+"_app", 1,0.02,travel_vel_,travel_acc_);
    }
    else
    {
        ROS_WARN_STREAM("Button color " << color << " not existent");
        return false;
    }

    return true;
}

bool task_board_tasks::center_slider()
{
    ur_ros_driver::SetCartTarget slider = cart_target("slider");
    ur_ros_driver::SetCartTarget slider_app = cart_target("slider_app");
    ur_ros_driver::SetCartTarget temp = slider_app;
    ur_ros_driver::SetGripper g_open = gripper_open();
    g_open.request.asynchronous = true;
    ur_ros_driver::SetGripper g_close = gripper_close(12,50,1);

    call_trajectory();
    call_gripper(g_open);

    add_cart_Path_Entry("slider_app", 1,blend_min_,travel_vel_,travel_acc_);
    add_cart_Path_Entry("slider", 1,blend_min_);
    call_trajectory();

    call_gripper(g_close);
    g_open.request.position = 24;
    call_gripper(g_open);


    // ROS_INFO_STREAM("finished Center Slider");
    // Für aktualisierung des Bildschirms
    ros::Duration(0.2).sleep();

    return true;
}


bool task_board_tasks::move_slider()
{
    geometry_msgs::Transform base_taskboard = tfBuffer_->lookupTransform("base","task_board",ros::Time(0)).transform;
    ur_ros_driver::SetCartTarget slider_app = cart_target("slider_app");
    ur_ros_driver::SetForceTarget forcemode = force_target(true,{1,1,0,0,0,0});
    forcemode.request.frame = base_taskboard;
    Eigen::Isometry3d base_taskboard_iso;
    tf::transformMsgToEigen(base_taskboard,base_taskboard_iso);

    robothon2023::GetTriangles srv;

    int cnt = 0;
    int cntFalse = 0;
    int fail_cnt = 0;
    double offset = 0.0;
    int dir = 0;
   
    while(call_triangles(srv)  && ros::ok())
    {
        if(srv.response.status == 1)
        {   
            double delta = srv.response.delta;
            int sign = (0 < delta) - (delta < 0);
            double force = delta*2.0 + sign*4;

            // std::cout << "delta: " << delta << " / force: " << force << std::endl;

            forcemode.request.IO = true;
            forcemode.request.wrench = {0,force,0,0,0,0};
            call_force_target(forcemode);
            ros::Duration(0.1).sleep();
        }
        else{
            if(srv.response.status == -1){
                if(fail_cnt == 3)
                {
                    // ROS_WARN("Starting Slider Fallback");
                    slider_fallback();
                    break;
                }
                else if(fail_cnt < 3)
                {
                    fail_cnt++;
                    // ROS_WARN("Exit Status Triangle -1");
                }
            } else if(srv.response.status == 2){
               //ROS_GREEN_STREAM("Exit Status Triangle Done" );
                break;
            } else {
                // ROS_WARN("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
            }
        }
    }

    forcemode.request.IO = false;
    call_force_target(forcemode);

    add_cart_Path_Entry("slider_app", 1,blend_min_,travel_vel_,travel_acc_);
    // ROS_INFO_STREAM("finished Move Slider");
    return true;
}

bool task_board_tasks::slider_fallback()
{
    geometry_msgs::Transform base_taskboard = tfBuffer_->lookupTransform("base","task_board",ros::Time(0)).transform;
    ur_ros_driver::SetForceTarget forcemode = force_target(true,{1,1,0,0,0,0});
    forcemode.request.frame = base_taskboard;
    forcemode.request.IO = true;
    forcemode.request.wrench = {0,-5,0,0,0,0};
    call_force_target(forcemode);

    auto start = ros::Time::now();
    do
    {
        ros::Duration(0.2).sleep();
    } while (wrench_.wrench.force.x > -4 
             && ros::ok() 
             && (ros::Time::now()-start) < ros::Duration(10));
    
    forcemode.request.wrench = {0,5,0,0,0,0};
    call_force_target(forcemode);

    start = ros::Time::now();
    do
    {
        ros::Duration(0.2).sleep();
    } while (wrench_.wrench.force.x < 4 
             && ros::ok()
             && (ros::Time::now()-start) < ros::Duration(10));
    return true;
}

bool task_board_tasks::move_plug(std::string from, std::string to)
{
    ur_ros_driver::SetCartTarget jack_to_app = cart_target("jack_"+ to +"_app");
    ur_ros_driver::SetCartTarget jack_from = cart_target("jack_"+ from,1);
    ur_ros_driver::SetForceTarget jack_to = force_target(true,{0,0,1,0,0,0}, {0,0,-7,0,0,0});
    ur_ros_driver::SetGripper g_open = gripper_open();
    g_open.request.asynchronous = true;
    ur_ros_driver::SetGripper g_close = gripper_close();

    call_trajectory();
    call_gripper(g_open);

    add_cart_Path_Entry("jack_"+ from +"_app", 1,0.01,travel_vel_,travel_acc_);
    add_cart_Path_Entry("jack_"+ from, 1,blend_min_);
    call_trajectory();

    call_gripper(g_close);

    add_cart_Path_Entry("jack_"+ from +"_app", 1,blend_min_,travel_vel_,travel_acc_);
    add_cart_Path_Entry("jack_"+ to +"_new_app", 1,blend_min_,travel_vel_,travel_acc_);
    call_trajectory();

    // ur_ros_driver::SetContactTarget contact = contact_target();
    // call_contact_target(contact);

    // call_force_target(jack_to);
    // while(wrench_.wrench.force.z < 3.5 && ros::ok())
    // {
    //     ros::Duration(0.5).sleep();
    //     continue;
    // }

    // if(!spiral_force(3,10,50,10))
    // {
    //     add_cart_Path_Entry("jack_"+ to +"_app", 0);
    //     call_trajectory();

    //     ur_ros_driver::SetContactTarget contact = contact_target();
    //     call_contact_target(contact);

    //     call_force_target(jack_to);
    //     if(!spiral_force(3,10,50,10,false))
    //     {
    //         call_gripper(g_open);
    //         return false;
    //     }
    // }

    geometry_msgs::Transform  jack_in = tfBuffer_->lookupTransform("base","jack_"+to+"_new",ros::Time(0)).transform;
    
    jack_to.request.wrench = {0,0,-20,0,0,0};
    call_force_target(jack_to);
    while(tcp_position_.transform.translation.z > jack_in.translation.z+0.001 && ros::ok())
    {
        ros::Duration(0.2).sleep();
        if(wrench_.wrench.force.z>20)
        {
            spiral_force(3,10,50,10,jack_in.translation.z+0.001);
        }
        continue;
    }
    jack_to.request.IO = false;
    call_force_target(jack_to);

    call_gripper(g_open);

    add_cart_Path_Entry("jack_"+ to +"_new_app", 1,0.01,travel_vel_,travel_acc_);
    call_trajectory();
    //call_gripper(g_close);

   //ROS_GREEN_STREAM("Finished Move Plug");

    return true;
}

bool task_board_tasks::grab_probe()
{
    ur_ros_driver::SetGripper g_open = gripper_open(100);
    g_open.request.asynchronous = true;
    ur_ros_driver::SetGripper g_close = gripper_close();

    call_trajectory();
    call_gripper(g_open);

    add_cart_Path_Entry("probe_pull_app", 1, 0.01,travel_vel_,travel_acc_);
    add_cart_Path_Entry("probe_pull", 1,blend_min_,travel_vel_,travel_acc_);
    call_trajectory();

    call_gripper(g_close);

    add_cart_Path_Entry("probe_plug", 1, blend_min_,travel_vel_,travel_acc_);
    //add_cart_Path_Entry("probe_plug_app", 1);
    add_cart_Path_Entry("probe_plug_app_helper", 1,0.01,travel_vel_,travel_acc_);
    
   //ROS_GREEN_STREAM("Finished Grab Probe");
    return true;
}

bool task_board_tasks::open_door()
{

    ur_ros_driver::SetForceTarget forcemode = force_target(true,{1,1,1,0,0,0},{9,0,-5,0,0,0});
    geometry_msgs::Transform  door_2= tfBuffer_->lookupTransform("base","door_2",ros::Time(0)).transform;
    geometry_msgs::Transform  task_board = tfBuffer_->lookupTransform("base","task_board",ros::Time(0)).transform;

    add_cart_Path_Entry("door_1_app", 0, 0.01,travel_vel_,travel_acc_);
    add_cart_Path_Entry("door_1", 1,blend_min_,travel_vel_,travel_acc_);
    call_trajectory();

    forcemode.request.frame = task_board;
    call_force_target(forcemode);
    while(pose_distance(door_2) > 0.005 && ros::ok())
    {
        ros::Duration(0.2).sleep();
        continue;
    }
    forcemode.request.IO = false;
    call_force_target(forcemode);

    add_cart_Path_Entry("door_3", 1, 0.01,travel_vel_,travel_acc_);
    add_cart_Path_Entry("door_4", 1, 0.01,travel_vel_,travel_acc_);
    add_cart_Path_Entry("door_5", 1, 0.01,travel_vel_,travel_acc_);
    
   //ROS_GREEN_STREAM("Finished Open Door");
    return true;
}

bool task_board_tasks::measure()
{
    
    ur_ros_driver::SetForceTarget measure = force_target(true,{0,0,1,0,0,0},{0,0,-6,0,0,0});

    add_cart_Path_Entry("meas_in_app", 1,blend_min_,travel_vel_,travel_acc_);
    call_trajectory();
    //ros::Duration(0.5).sleep();
    call_force_target(measure);
    geometry_msgs::Transform meas = tfBuffer_->lookupTransform("base","meas",ros::Time(0)).transform;

    ur_ros_driver::SetContactTarget contact = contact_target();
    call_contact_target(contact);

    //wait until tcp does not move anymore
    // do{
    //     ros::Duration(0.1).sleep();
    // } while(pose_distance(meas) > 0.001 && ros::ok());

    while(wrench_.wrench.force.z < 5 && ros::ok())
    {
        ros::Duration(0.1).sleep();
    } 

    measure.request.IO = false;
    call_force_target(measure);

    //ros::Duration(0.5).sleep();

    add_cart_Path_Entry("meas_in_app", 1,blend_min_,travel_vel_,travel_acc_);

   //ROS_GREEN_STREAM("Finished Measureing");
    return true;
}

bool task_board_tasks::return_probe()
{
    ur_ros_driver::SetForceTarget probe= force_target(true,{1,1,1,0,0,0},{0,-10,0,0,0,0});
    geometry_msgs::Transform  probe_target = tfBuffer_->lookupTransform("base","probe_pull",ros::Time(0)).transform;
    geometry_msgs::Transform  task_board = tfBuffer_->lookupTransform("base","task_board",ros::Time(0)).transform;
    probe.request.frame = task_board;

    ur_ros_driver::SetGripper g_open = gripper_open(50);
    ur_ros_driver::SetGripper g_close = gripper_close();

    add_cart_Path_Entry("probe_plug_app",1,0.01,travel_vel_,travel_acc_);
    add_cart_Path_Entry("probe_plug", 1,blend_min_,travel_vel_,travel_acc_);
    call_trajectory();

    call_force_target(probe);

    geometry_msgs::Transform oldTcp;
    //wait until tcp does not move anymore
    do{
        oldTcp = tcp_position_.transform;
        ros::Duration(0.5).sleep();
    } while(pose_distance(oldTcp) > 0.001 && ros::ok());

    probe.request.IO = false;
    call_force_target(probe);

    call_gripper(g_open);

    add_cart_Path_Entry("probe_pull_app", 1,blend_min_,travel_vel_,travel_acc_);

   //ROS_GREEN_STREAM("Finished Return Probe");
    return true;
}

bool task_board_tasks::get_hook(bool bring)
{
    ur_ros_driver::SetGripper g_close = gripper_close();
    ur_ros_driver::SetGripper g_open = gripper_open(50);
    if(!bring) {
        
        call_trajectory();
        call_gripper(g_open);
        if(config != 0)
            add_cart_Path_Entry("hook_grab_1_app_helper", 1,0.02,travel_vel_,travel_acc_); 
        add_cart_Path_Entry("hook_grab_1_app", 1,0.01,travel_vel_,travel_acc_);
        add_cart_Path_Entry("hook_grab_1", 1);
        call_trajectory();

        call_gripper(g_close);

        add_cart_Path_Entry("hook_grab_2", 1);
        add_cart_Path_Entry("hook_grab_3", 1,0.005);
        add_cart_Path_Entry("hook_grab_3_app", 1,0.005,travel_vel_,travel_acc_);
        //add_cart_Path_Entry("hook_grab_1_app", 0,0.02);
        call_trajectory();
        //hook_grab_1_app withz joints
        ur_ros_driver::SetJointTarget hook_grab_1_app_joints = joint_target({ 0.8822202682495117, -1.116662399177887,1.4039080778705042, -1.8579136333861292, -1.5679720083819788,-0.6286428610431116});
        call_joint_target(hook_grab_1_app_joints);
        // [1.403543774281637, -1.1164595645717164, 0.8823729753494263, -1.8575102291502894, -1.5687769095050257, -0.6286428610431116]


    } else {
        if(config != 0)
            add_cart_Path_Entry("hook_grab_1_app", 1,0.02,travel_vel_,travel_acc_); 
        add_cart_Path_Entry("hook_grab_3_app", 1,0.005,travel_vel_,travel_acc_);
        add_cart_Path_Entry("hook_grab_3", 1, 0.005);
        add_cart_Path_Entry("hook_grab_2", 1, blend_min_);
        add_cart_Path_Entry("hook_grab_1", 1, blend_min_);
        call_trajectory();

        call_gripper(g_open);

        add_cart_Path_Entry("hook_grab_1_app", 1,blend_min_,travel_vel_,travel_acc_);
    }

   //ROS_GREEN_STREAM("Finished Get Hook");
    return true;
}


bool task_board_tasks::get_hook_new(bool bring)
{
    ur_ros_driver::SetGripper g_close = gripper_close();
    ur_ros_driver::SetGripper g_open = gripper_open(100);
    ur_ros_driver::SetGripper g_open_1 = gripper_open(30);
    if(!bring) 
    {
        call_trajectory();
        call_gripper(g_open_1);
        if(config != 0) 
        {
            add_cart_Path_Entry("hook_grab_1_app_helper", 1,0.02,travel_vel_,travel_acc_);
        }
        //fix joint position 
        
    
        //array6d hook_grab_1_app_joints({0.8822202682495117, -1.116662399177887,1.4039080778705042, -1.8579136333861292, -1.5679720083819788,2.507723331451416});
        //add_joint_Path_Entry(hook_grab_1_app_joints, 0,0, joint_vel_ , joint_acc_);
        
        array6d hook_grab_1_app_joints({0.8822202682495117, -1.116662399177887,1.4039080778705042, -1.8579136333861292, -1.5679720083819788,-0.6286428610431116});
        add_joint_Path_Entry(hook_grab_1_app_joints, 0,0.05, joint_vel_ , joint_acc_);
    
        //call_trajectory();
        //std::cout << "press enter to continue" << std::endl;
        //std::cin.ignore();
            
        add_cart_Path_Entry("hooknew_grab_short", 1,0.02,travel_vel_,travel_acc_);
        add_cart_Path_Entry("hooknew_grab", 1,blend_min_,travel_vel_,travel_acc_);
        call_trajectory();

        call_gripper(g_close);

        add_cart_Path_Entry("hooknew_grab_short", 1, blend_min_,travel_vel_,travel_acc_);
        
        if (config ==1)
        {
            array6d hook_grab_1_app_joints({0.8822202682495117, -1.116662399177887,1.4039080778705042, -1.8579136333861292, -1.5679720083819788,-0.6286428610431116});
            add_joint_Path_Entry(hook_grab_1_app_joints, 0,0.02, joint_vel_ , joint_acc_);
        }
        else
        {
            add_cart_Path_Entry("hooknew_grab_app", 1, 0.04, travel_vel_, travel_acc_);
        }


        //call_trajectory();
        //std::cout << "press enter to continue" << std::endl;
        //std::cin.ignore();
        
        

        // [1.403543774281637, -1.1164595645717164, 0.8823729753494263, -1.8575102291502894, -1.5687769095050257, -0.6286428610431116]
        


    } 
    else 
    {
        
        add_cart_Path_Entry("hooknew_grab_app", 1,0.02,travel_vel_,travel_acc_);
        add_cart_Path_Entry("hooknew_grab_short", 1,0.02,travel_vel_,travel_acc_);
        add_cart_Path_Entry("hooknew_grab", 1,blend_min_,travel_vel_,travel_acc_);
        
        call_trajectory();

        call_gripper(g_open);

        add_cart_Path_Entry("hooknew_grab_app", 1,0.02,travel_vel_,travel_acc_);
    }

   //ROS_GREEN_STREAM("Finished Get Hook");

    return true;
}

bool task_board_tasks::hook_cable()
{
    add_cart_Path_Entry("cable_1_app", 1, 0.002,travel_vel_,travel_acc_);
    add_cart_Path_Entry("cable_1", 1, 0.002);
    add_cart_Path_Entry("cable_2", 1, 0.002);
    add_cart_Path_Entry("cable_3", 1, 0.002);
    add_cart_Path_Entry("cable_4", 1, 0.002);
    add_cart_Path_Entry("cable_5", 1, 0.002);

   //ROS_GREEN_STREAM("Finished Hook Cable");
    return true;
}

bool task_board_tasks::wind_cable()
{
    //#define fast_mode

    for(int i=0;i<2;i++)
    {
        add_cart_Path_Entry("wind_1", 1, 0.005,wind_vel_,wind_acc_);
        add_cart_Path_Entry("wind_3", 1, 0.005,wind_vel_,wind_acc_);
        add_cart_Path_Entry("wind_4", 1, 0.005,wind_vel_,wind_acc_);
        add_cart_Path_Entry("wind_51", 1, 0.002,wind_vel_,wind_acc_);
        add_cart_Path_Entry("wind_61", 1, 0.002,wind_vel_,wind_acc_);
        add_cart_Path_Entry("wind_7", 1, 0.005,wind_vel_,wind_acc_);
        add_cart_Path_Entry("wind_8", 1, 0.005,wind_vel_,wind_acc_);
        #ifdef fast_mode
            if(i==0){press_button_hook();}
        #endif
    }
    // add_cart_Path_Entry("wind_9", 1, 0.002,wind_vel_,wind_acc_);
    // add_cart_Path_Entry("wind_10", 1, 0.002,wind_vel_,wind_acc_);

    add_cart_Path_Entry("wind_18", 1, 0.001,wind_vel_,wind_acc_);
    add_cart_Path_Entry("wind_19", 1, 0.001,wind_vel_,wind_acc_);

    add_cart_Path_Entry("wind_20", 1, 0.002,wind_vel_,wind_acc_); //nicht benötigt
    add_cart_Path_Entry("wind_21", 1, 0.002,wind_vel_,wind_acc_);//nicht benötigt
    
   //ROS_GREEN_STREAM("Finished Wind Cable");
    return true;
}


bool task_board_tasks::home(bool fast)
{
   
    ur_ros_driver::SetGripper g_close = gripper_close();
    
    double blend = 0;

    call_trajectory();
    call_gripper(g_close);

    if(fast == true)
    {
        blend = 0.02;
    }

    if(config == 0)
    {
        array6d home_target({1.0663893222808838, -1.9157773456969203, 2.2702224890338343, -1.912236829797262, -1.5635083357440394, 2.7358269691467285});
        add_joint_Path_Entry(home_target,0, blend, joint_vel_ , joint_acc_);
        std::cout << "config 0 home" << std::endl;
    }
    else if(config ==1)
    {
        //array6d home_target({1.0695934295654297, -1.930535455743307, 2.2863615194903772, -1.9397150478758753, -1.567730728779928, -2.079547707234518});
        array6d home_target({1.0695934295654297, -1.930535455743307, 2.2863615194903772, -1.9397150478758753, -1.567730728779928, 4.244777202606201});
        add_joint_Path_Entry(home_target,0, blend, joint_vel_ , joint_acc_);
        std::cout << "config 1 home" << std::endl;
    }
    else if(config == 2)
    {
        array6d home_target({1.0695934295654297, -1.930535455743307, 2.2863615194903772, -1.9397150478758753, -1.567730728779928, -0.5293334166156214});
        add_joint_Path_Entry(home_target,0, blend, joint_vel_ , joint_acc_);
        std::cout << "config 2 home" << std::endl;
    }
    else if(config == 3)
    {
        array6d home_target({1.0695934295654297, -1.930535455743307, 2.2863615194903772, -1.9397150478758753, -1.567730728779928, 1.089403748512268});
        add_joint_Path_Entry(home_target,0, blend, joint_vel_ , joint_acc_);
        std::cout << "config 1 home" << std::endl;
    }
    else
    {
        return false;
    }

    if (fast == false)
    {
        call_trajectory();
        // std::cout << "turn" << std::endl;
    }
    return true;
}

/**
 * @brief 
 * 
 * @param start minimal force 
 * @param stop maximal force
 * @param step_c steps in one rotation
 * @param step_f amount of rotations
 * @return true 
 * @return false 
 */
bool task_board_tasks::spiral_force(double start, double stop, int step_c , double step_f, double z_limit, bool dir)
{
    double radius = start;
    double inc = (2* M_PI) / step_c;
    double inc_f = (stop - start)/(step_f*step_c); 

    int counter;
    double x,y;
    ur_ros_driver::SetForceTarget force_srv = force_target(true,{1,1,1,0,0,0},{0,0,0,0,0,0});
    // ROS_INFO_STREAM("Bedingung: "<< (radius <= stop && tcp_position_.transform.translation.z >= 0.109) || wrench_.wrench.force.z < 7);
    
    while ((radius <= stop && tcp_position_.transform.translation.z >= z_limit) || wrench_.wrench.force.z < 3)
    {
        if(dir)
        {
            double x = radius*sin(inc*counter);
            double y = radius*cos(inc*counter);
        }
        else
        {
            double x = radius*sin(-inc*counter);
            double y = radius*cos(-inc*counter);
        }
        double z = -20;
        force_srv.request.wrench = {x,y,z,0,0,0};
        call_force_target(force_srv);
        counter +=1;
        radius += inc_f;
        //ROS_INFO_STREAM("x: "<< x << " / y: " << y);
        ros::Duration(0.05).sleep();
    }
    force_srv.request.IO = false;
    call_force_target(force_srv);

    if(tcp_position_.transform.translation.z >= z_limit)
        return false;
    else
        return true;
}

double task_board_tasks::pose_distance(geometry_msgs::Transform target)
{
    return std::sqrt(std::pow(target.translation.x - tcp_position_.transform.translation.x,2) 
                   + std::pow(target.translation.y - tcp_position_.transform.translation.y,2) 
                   + std::pow(target.translation.z - tcp_position_.transform.translation.z,2));
}

bool task_board_tasks::press_button_hook()
{
    ur_ros_driver::SetGripper g_close = gripper_close();

    add_cart_Path_Entry("btn_red_hook_app", 1, 0.005,travel_vel_,travel_acc_);
    add_cart_Path_Entry("btn_red_hook_app2", 1,blend_min_,travel_vel_,travel_acc_);
    call_trajectory();

    //ur_ros_driver::SetContactTarget contact = contact_target();
    //call_contact_target(contact);

    ur_ros_driver::SetForceTarget btn = force_target(true,{0,0,1,0,0,0},{0,0,-10,0,0,0});
    call_force_target(btn);
    while(wrench_.wrench.force.z < 5 && ros::ok())
    {
        ros::Duration(0.1).sleep();
        continue;
    }
    btn.request.IO = false;
    call_force_target(btn);

    add_cart_Path_Entry("btn_red_hook_app2", 1,0.01);
    add_cart_Path_Entry("btn_red_hook_app", 1, 0.005);

    return true;

}

bool task_board_tasks::check_finish()
{
    add_cart_Path_Entry("btn_red_hook_2slider", 1, 0.005,travel_vel_,travel_acc_);
    add_cart_Path_Entry("slider_hook", 1,0.005,travel_vel_,travel_acc_);
    call_trajectory();
    if(!call_finish_detection())
    {
        return false;
    }
    return true;
}

bool task_board_tasks::calculate_config()
{
    tf2::Quaternion rot_quat;
    tf2::fromMsg(tfBuffer_->lookupTransform("base","task_board",ros::Time(0)).transform.rotation,rot_quat);
    tf2::Vector3 rpy;
    tf2::Matrix3x3 rot(rot_quat);
    rot.getRPY(rpy[0], rpy[1], rpy[2]);
    auto euler = abs(rpy[2]*180/M_PI - 180);
    config = int((euler-45)/90)%4;
    return true;
}

bool task_board_tasks::function_test()
{
    //ur_ros_driver::SetContactTarget contact = contact_target();
    //call_contact_target(contact);
    //std::cout << "Contact" << std::endl;

    ur_ros_driver::SetForceTarget forcemode = force_target(1,{0,1,0,0,0,0},{0,0,-5,0,0,0});
    forcemode.request.frame = tcp_position_.transform;
    ur_ros_driver::SetContactTarget contact = contact_target();
    call_contact_target(contact);
    // call_force_target(forcemode);

    // while(wrench_.wrench.force.z < 5 && ros::ok())
    // {
    //     continue;
    // }
    forcemode.request.IO = 0;
    call_force_target(forcemode);

    return true;
}