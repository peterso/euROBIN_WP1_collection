/**
 * @file point_recording.cpp
 * @authors Adrian Müller (adrian.mueller@study.thws.de), 
 *          Maximilian Hornauer (maximilian.hornauer@study.thws.de),
 *          Usama Ali (usama.ali@study.thws.de)
 * @brief Program for recording points by using freedrive
 * @version 0.1
 * @date 2023-03-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <filesystem>

#include <ros/ros.h>
#include <ros/package.h>

#include <fstream>
#include <termios.h>

#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Joy.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <boost/array.hpp>

#include <ur_ros_driver/SetFreedrive.h>
#include <ur_ros_driver/SetCartTarget.h>
#include <robothon2023/AddTf2.h>
#include <geometry_msgs/TransformStamped.h>
#include <ur_ros_driver/StartJog.h>
#include <ur_ros_driver/JogControl.h>
#include <ur_ros_driver/SetForceTarget.h>
#include <ur_ros_driver/SetGripper.h>
#include <tf2_ros/transform_listener.h>

boost::array<long int, 6> free_axes = {1, 1, 1, 1, 1, 1};

geometry_msgs::TransformStamped tcp_pose;

void main_menu();
void free_drive_menu();
void con_menu();
void jog_menu(int frame, double speed, double step);

void tcpCallback(const geometry_msgs::TransformStamped& msg)
{
    tcp_pose = msg;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "point_recorder");
    ros::NodeHandle n;

    ros::ServiceClient client_g = n.serviceClient<ur_ros_driver::SetGripper>("/set_gripper");
    ros::ServiceClient client_f = n.serviceClient<ur_ros_driver::SetFreedrive>("/set_freedive");
    ros::ServiceClient client_fm = n.serviceClient<ur_ros_driver::SetForceTarget>("/set_force_mode");
    ros::ServiceClient client_t = n.serviceClient<ur_ros_driver::SetCartTarget>("/set_cart_target");
    ros::ServiceClient client_tf = n.serviceClient<robothon2023::AddTf2>("/store_tf");
    ros::ServiceClient jog_client = n.serviceClient<ur_ros_driver::StartJog>("/start_jog");
    
    ros::Subscriber sub = n.subscribe("/tcp_pose", 1000, tcpCallback);

    ros::Publisher jog_control_pub = n.advertise<ur_ros_driver::JogControl>("/jog_control",1000);
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    robothon2023::AddTf2 addTF_srv;
    ur_ros_driver::SetFreedrive freedrive_srv;
    freedrive_srv.request.IO = true;
    freedrive_srv.request.free_axes = free_axes;
    ur_ros_driver::SetCartTarget target_srv;
    target_srv.request.mode = 1;
    target_srv.request.speed = 0.25;
    target_srv.request.acceleration = 0.25;
    target_srv.request.asynchronous = false;
    ur_ros_driver::SetForceTarget force_srv;
    force_srv.request.type = 2;
    force_srv.request.limits = {1,1,1,1,1,1};
    ur_ros_driver::SetGripper gripper_srv;
    gripper_srv.request.force = 0;
    gripper_srv.request.asynchronous = false;
    gripper_srv.request.speed = 50;

    int main;
    bool relation;
    double offset;
    std::string point_ID;

    client_f.call(freedrive_srv);

    do{
        main_menu();
        std::cin >> main;

        switch (main)
        {
        case 1:
            {
                std::cout << "Punktname: ";
                std::cin >> point_ID;
                std::cout << "Add z Offset ? [mm] ";
                std::cin >> offset;
                std::cout << "Is this Pose in relation to the target ? [1/0] ";
                std::cin >> relation;
                addTF_srv.request.pose = tcp_pose;
                addTF_srv.request.pose.header.stamp = ros::Time::now();
                addTF_srv.request.pose.header.frame_id = "base";
                addTF_srv.request.pose.child_frame_id = point_ID;
                addTF_srv.request.relative = relation;
                if(relation)
                {
                    addTF_srv.request.pose.header.frame_id = "task_board";
                }
                client_tf.call(addTF_srv);
                if(offset != 0) {
                    addTF_srv.request.pose.transform = geometry_msgs::Transform();
                    addTF_srv.request.pose.transform.translation.z = -1 * offset / 1000;
                    addTF_srv.request.pose.transform.rotation.w = 1;
                    addTF_srv.request.pose.header.frame_id = point_ID;
                    addTF_srv.request.pose.child_frame_id = point_ID + "_app";
                    addTF_srv.request.relative = false;
                    client_tf.call(addTF_srv);
                }
                break;
            }
        case 2:
            {
                free_drive_menu();

                int axis;
                std::cin >> axis;

                freedrive_srv.request.IO = false;
                client_f.call(freedrive_srv);

                if(axis < 4 && axis > 0)
                {
                    freedrive_srv.request.feature = axis-1;
                    if(axis == 3) freedrive_srv.request.custom_frame = tfBuffer.lookupTransform("base", "task_board",ros::Time(0)).transform;
                }

                freedrive_srv.request.IO = true;
                freedrive_srv.request.free_axes = free_axes;
                client_f.call(freedrive_srv);
                break;
            }
        case 3:
            {
                con_menu();
                int con;
                std::cin >> con;
                switch (con)
                {
                case 0:
                    break;
                case 1:
                    std::cout << "Put in values for x,y,z,r,p,y (1/0)" << std::endl;
                    for(int i=0;i<6;i++){std::cin >> free_axes[i];}
                    break;
                case 2:
                    std::cout << "Put in values for x,y,z (1/0)" << std::endl;
                    for(int i=0;i<3;i++){std::cin >> free_axes[i];}
                    break;
                case 3:
                    std::cout << "Put in values for r,p,y (1/0)" << std::endl;
                    for(int i=3;i<6;i++){std::cin >> free_axes[i];}
                    break;
                
                default:
                    break;
                }
                //rtde->freedrive(0);
                //rtde->freedrive(1,free_axes);
                freedrive_srv.request.IO = false;
                client_f.call(freedrive_srv);
                freedrive_srv.request.IO = true;
                freedrive_srv.request.free_axes = free_axes;
                client_f.call(freedrive_srv);
                break;
            }
        case 4:
            {  
                freedrive_srv.request.IO = false;
                client_f.call(freedrive_srv);
                geometry_msgs::Transform taskboard = tfBuffer.lookupTransform("base", "task_board" ,ros::Time(0)).transform; 
                bool running = true;
                int feature = 1;
                //double step = 0.001;
                double step = 0.001;
                double stepIncrement = 0.001;
                int time = 1;

                //ur_ros_driver::JogControl msg;
                //ur_ros_driver::StartJog srv;
                //srv.request.IO = true;
                //jog_client.call(srv);

                int kfd = 0;
                struct termios cooked, raw;
                tcgetattr(kfd, &cooked);
                memcpy(&raw, &cooked, sizeof(struct termios));
                
                raw.c_lflag &=~ (ICANON | ECHO);
                // Setting a new line, then end of file                         
                raw.c_cc[VEOL] = 1;
                raw.c_cc[VEOF] = 2;
                tcsetattr(kfd, TCSANOW, &raw);

                //system("stty raw -echo");
                jog_menu(feature, step, stepIncrement);

                while(ros::ok() && running){
                    Eigen::Matrix4d deltaP = Eigen::Matrix4d::Identity();
                    
                    //char c = getc(stdin);
                    char c;
                    if(read(kfd, &c, 1) < 0)
                    {
                      c = '0';
                    }
                    ROS_INFO("value: 0x%02X\n", c);
                    //ros::Duration(2).sleep();
                    switch (c)
                    {
                    case '0':
                        //system("stty cooked echo");
                        tcsetattr(kfd, TCSANOW, &cooked);
                        running = false;
                        break;
                    case 'a':
                        deltaP(0,3) -= step;
                        break;
                    case 'd':
                        deltaP(0,3) += step;
                        break;
                    case 's':
                        deltaP(1,3) -= step;
                        break;
                    case 'w':
                        deltaP(1,3) += step;
                        break;
                    case 'e':
                        deltaP(2,3) -= step;
                        break;
                    case 'q':
                        deltaP(2,3) += step;
                        break;        
                    case 'j':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(-step * M_PI, Eigen::Vector3d::UnitX()).matrix();
                        break;
                    case 'l':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(step * M_PI, Eigen::Vector3d::UnitX()).matrix();
                        break;
                    case 'k':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(-step * M_PI, Eigen::Vector3d::UnitY()).matrix();
                        break;
                    case 'i':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(step * M_PI, Eigen::Vector3d::UnitY()).matrix();
                        break;
                    case 'o':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(-step * M_PI, Eigen::Vector3d::UnitZ()).matrix();
                        break;
                    case 'u':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(step * M_PI, Eigen::Vector3d::UnitZ()).matrix();
                        break;
                    case 'n':
                        step += stepIncrement;
                        break;
                    case 'm':
                        step -= stepIncrement;
                        break;
                    case '1':
                        stepIncrement = 0.001;
                        // stepIncrement = 1;
                        // time = 0.1;
                        break;
                    case '2':
                        stepIncrement = 0.005;
                        // stepIncrement = 2.5;
                        // time = 0.2;
                        break;
                    case '3':
                        stepIncrement = 0.01;
                        // time = 0.3;
                        // stepIncrement = 5;
                        break;
                    case '4':
                        stepIncrement = 0.025;
                        // time = 0.5;
                        // stepIncrement = 7.5;
                        break;
                    case 'b':
                        feature = (feature+1) %2;
                        break;
                    default:
                        break;
                    }
                    //msg.stamp = ros::Time::now();
                    //msg.feature = feature;
                    //msg.speeds = speeds;
                    //jog_control_pub.publish(msg);
                    
                    //std::cout << "set Frame" << std::endl;
                    geometry_msgs::Transform null;
                    null.rotation.w = 0;
                    null.rotation.x = 0;
                    null.rotation.y = 0;
                    null.rotation.z = 0;
                    Eigen::Isometry3d robotPose_Iso;
                    tf::transformMsgToEigen(tcp_pose.transform, robotPose_Iso);
                    Eigen::Matrix4d robotPose_Mat = robotPose_Iso.matrix();
                    Eigen::Matrix4d resPose; 
                    if(feature == 0){
                        resPose = robotPose_Mat;
                        resPose(0,3) += deltaP(0, 3);
                        resPose(1,3) += deltaP(1, 3);
                        resPose(2,3) += deltaP(2, 3);
                        resPose.block<3,3>(0,0) = deltaP.block<3,3>(0,0) * resPose.block<3,3>(0,0);
                    }
                    else if(feature == 1){
                        resPose = robotPose_Mat * deltaP;
                    }
                    else if(feature == 2) {//TODO
                        Eigen::Isometry3d taskboard_Iso;
                        tf::transformMsgToEigen(taskboard, taskboard_Iso);
                        Eigen::Matrix4d taskboard_rot = Eigen::Matrix4d::Identity();
                        taskboard_rot.block<3,3>(0,0) = taskboard_Iso.matrix().block<3,3>(0,0);

                        deltaP = taskboard_rot*deltaP;


                        resPose = robotPose_Mat;
                        resPose(0,3) += deltaP(0, 3);
                        resPose(1,3) += deltaP(1, 3);
                        resPose(2,3) += deltaP(2, 3);
                        resPose.block<3,3>(0,0) = deltaP.block<3,3>(0,0) * resPose.block<3,3>(0,0);
                    }
                    
                    tf::transformEigenToMsg(Eigen::Isometry3d(resPose), target_srv.request.cartesian_goal);
                    client_t.call(target_srv);
                    
                    jog_menu(feature, step, stepIncrement);
                    //ros::Duration(0.05).sleep();
                }
                
                //srv.request.IO = false;
                //jog_client.call(srv);
                force_srv.request.IO = false;
                client_fm.call(force_srv);
                ros::Duration(1.0).sleep();
                freedrive_srv.request.IO = true;
                client_f.call(freedrive_srv);
                break;
            
            }
            case 5:
            {
                freedrive_srv.request.IO = false;
                client_f.call(freedrive_srv);

                system("clear");
                std::cout << "------------------------" << std::endl;
                std::cout << "Move to TF" << std::endl;
                std::cout << "------------------------" << std::endl;
                std::cout << "Please put name of target frame in" << std::endl;
                std::string tf;
                std::cin >> tf;
                try{
                target_srv.request.cartesian_goal = tfBuffer.lookupTransform("base", tf ,ros::Time(0)).transform;
                }
                catch(tf2::TransformException &ex)
                {
                    ROS_WARN_STREAM(ex.what());
                    ros::Duration(2.0).sleep();
                    freedrive_srv.request.IO = true;
                    client_f.call(freedrive_srv);
                    break;
                }
                //std::cout << "1 - execute" << std::endl;
                //std::cout << "0 - break" << std::endl;
                //std::cout << "------------------------" << std::endl;
                bool execute = true;
                //std::cin >> execute;
                

                if(execute)
                    client_t.call(target_srv);

                freedrive_srv.request.IO = true;
                client_f.call(freedrive_srv);
                break;

            }
            case 6:
            {
                system("clear");
                std::cout << "------------------------" << std::endl;
                std::cout << "Gripper" << std::endl;
                std::cout << "------------------------" << std::endl;
                std::cout << "1 - Gripper Open" << std::endl;
                std::cout << "2 - Gripper Close" << std::endl;
                std::cout << "0 - break" << std::endl;
                std::cout << "------------------------" << std::endl;
                int input;
                std::cin >> input;

                switch (input)
                {
                case 0:
                    break;
                case 1:
                    gripper_srv.request.position = 100;
                    gripper_srv.request.force = 50;
                    break;
                case 2:
                    gripper_srv.request.position = 0;
                    gripper_srv.request.force = 100;
                    break;
                
                default:
                    break;
                }

                client_g.call(gripper_srv);
            }
        default:
            break;
        }

    }while(ros::ok() && main != 0 );

    freedrive_srv.request.IO = false;
    client_f.call(freedrive_srv);

}


void main_menu()
{   
    system("clear");
    std::cout << "------------------------" << std::endl;
    std::cout << "Main Menu" << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "Please make your selection" << std::endl;
    std::cout << "1 - save point" << std::endl;
    std::cout << "2 - set axis" << std::endl;
    std::cout << "3 - set constrains" << std::endl;
    std::cout << "4 - move remotely" << std::endl;
    std::cout << "5 - move to tf" << std::endl;
    std::cout << "6 - move Gripper" << std::endl;
    std::cout << "0 - Quit" << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "Current Position: " << std::endl;
    std::cout << "\rX: " << tcp_pose.transform.translation.x << "\t Y: " << tcp_pose.transform.translation.y << "\t Z: " << tcp_pose.transform.translation.z << std::endl; 
    std::cout << "\rRX: " << tcp_pose.transform.rotation.x << "\t RY: " << tcp_pose.transform.rotation.y << "\t RZ: "<< tcp_pose.transform.rotation.z << "\t RW: "<< tcp_pose.transform.rotation.w << std::endl;
    std::cout << std::endl;
    std::cout << "Current Constrains: ";
    for(int i=0; i<6; i++){std::cout << free_axes[i] << ", ";}
    std::cout << std::endl;

    std::cout << "------------------------" << std::endl;
    std::cout << "Selection: ";
}

void free_drive_menu()
{
    system("clear");
    std::cout << "------------------------" << std::endl;
    std::cout << "Freedrive Frame" << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "1 - Base" << std::endl;
    std::cout << "2 - TCP" << std::endl;
    std::cout << "3 - Taskboard" << std::endl;
    std::cout << "0 - back" << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "Selection: ";
}

void con_menu()
{
    system("clear");
    std::cout << "------------------------" << std::endl;
    std::cout << "Freedrive Constrains" << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "Please choose the constrains" << std::endl;
    std::cout << "1 - all" << std::endl;
    std::cout << "2 - position" << std::endl;
    std::cout << "3 - orientaion" << std::endl;
    std::cout << "0 - back" << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "Selection: ";
}

void jog_menu(int feature, double speed, double step)
{
    std::vector<std::string> frames({"Base", "Tool", "Task Board"});
    system("clear");
    std::cout << "------------------------" << std::endl;
    std::cout << "\rRobot remote control" << std::endl << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "\rControls:" << std::endl;
    std::cout << "\r\tTranslation:" << std::endl;
    std::cout << "\r\t\tX: + a   - d" << std::endl;
    std::cout << "\r\t\tY: + w   - s" << std::endl;
    std::cout << "\r\t\tZ: + q   - e" << std::endl;
    std::cout << "\r\tRotation:" << std::endl;
    std::cout << "\r\t\tX: + j   - l" << std::endl;
    std::cout << "\r\t\tY: + i   - k" << std::endl;
    std::cout << "\r\t\tZ: + u   - o" << std::endl;
    std::cout << "\r\tSpeed: Up n, Down m" << std::endl;
    std::cout << "\r\tStep: 1mm:1, 5mm:2, 10mm:3, 25mm:4" << std::endl;
    std::cout << "\r\tSwitch Frame: b" << std::endl;
    std::cout << "\r\tQuit: 0" << std::endl;
    std::cout << "------------------------" << std::endl;
    //std::cout << std::endl << std::endl<< std::endl;
    std::cout << "\rX: " << tcp_pose.transform.translation.x << "\t Y: " << tcp_pose.transform.translation.y << "\t Z: " << tcp_pose.transform.translation.z << std::endl <<"\rRX: " << tcp_pose.transform.rotation.x << "\t RY: " << tcp_pose.transform.rotation.y << "\t RZ: "<< tcp_pose.transform.rotation.z << "\t RW: "<< tcp_pose.transform.rotation.w << std::endl;
    std::cout << "\rSpeed: " << speed << "\tStep: "<< step << std::endl;
    std::cout << "\rFrame: " << frames[feature] << std::endl;
    std::cout << "------------------------" << std::endl;
}

