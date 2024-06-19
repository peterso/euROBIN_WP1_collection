/**
 * @file task_board_scheduler.cpp
 * @authors Adrian Müller (adrian.mueller@study.thws.de), 
 *          Maximilian Hornauer (maximilian.hornauer@study.thws.de),
 *          Usama Ali (usama.ali@study.thws.de)
 * @brief Program to schedule tasks
 * @version 0.1
 * @date 2023-04-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <robothon2023/task_board_tasks.hpp>

#include <ur_ros_driver/ros_color_stream.hpp>

const int btn_blue = 1;
const int slider = 2;
const int plug = 3;
const int measure = 4;
const int wind_cable = 5;
const int btn_red = 6;
const int home = 0;


bool checkSequence(std::vector<int> sequence){
    //measure
    auto plug_it = std::find(sequence.begin(), sequence.end(), plug);
    auto measure_it = std::find(sequence.begin(), sequence.end(), measure);
    auto wind_it = std::find(sequence.begin(), sequence.end(), wind_cable);

    if(wind_it < measure_it) return false;
    if(measure_it < plug_it) return false;
    return true;
}

int main(int argc, char* argv[])
{
    bool hook = false;
    bool probe = false;

    int ctn = 0;

    ros::init(argc, argv, "taskboard_scheduler");
    ros::NodeHandle n("~");
    ros::AsyncSpinner spinner(2); 
    spinner.start();


    std::vector<int> sequenz = {1,2,3,4,5,6};

    task_board_tasks task = task_board_tasks(n);

    if(!checkSequence(sequenz)) {
       ROS_ERROR("SEQUENCE INVALID!");
       return 1;
    }

    task.home();
    ros::Duration(2).sleep();
    task.calculate_config();
    
    // while(!task.call_board_detection() && ros::ok());
    std::cout << "Press ENTER to start" << std::endl;
    std::cin.ignore();
    auto start = std::chrono::high_resolution_clock::now();
    task.call_robot_time();
    /** ENABLE THIS TO LOG DATA TO FILE **/
    // task.call_log(true,"log_2");
    
    // move to optimal start position
    task.home();
    
    int counter = 0;
    sequenz.push_back(0);
    do{
        switch (sequenz[counter])
        {
        case btn_blue:
            ROS_MAGENTA_STREAM("Task: btn blue");
            task.call_robot_time();
            task.press_button("blue");
            counter++;
            break;
        case slider:
            ROS_MAGENTA_STREAM("Task: slider");
            task.call_robot_time();
            task.center_slider();
            task.move_slider();
            counter++;
            break;
        case plug:
            ROS_MAGENTA_STREAM("Task: plug");
            task.call_robot_time();
            task.move_plug("black","red");
            counter++;
            break;
        case measure:
            ROS_MAGENTA_STREAM("Task: grab probe");
            task.call_robot_time();
            if(task.grab_probe())
            {
                probe = true;
            }

            ROS_MAGENTA_STREAM("Task: open door");
            task.call_robot_time();
            task.open_door();

            ROS_MAGENTA_STREAM("Task: measure");
            task.call_robot_time();
            task.measure();

            ROS_MAGENTA_STREAM("Task: return probe");
            task.call_robot_time();
            if(task.return_probe())
            {
                probe = false;
            }
            counter++;
            break;
        case wind_cable:
            ROS_MAGENTA_STREAM("Task: get hook");
            task.call_robot_time();
            if(task.get_hook_new())
            {
                hook = true;
            }

            ROS_MAGENTA_STREAM("Task: hook cable");
            task.call_robot_time();
            task.hook_cable();

            ROS_MAGENTA_STREAM("Task: wind cable");
            task.call_robot_time();
            task.wind_cable();
            counter++;
            break;
        case btn_red:
            ROS_MAGENTA_STREAM("Task: btn red");
            task.call_robot_time();
            if(hook){
                task.press_button_hook();
            } else {
                task.press_button("red");
            }
            counter++;
            break;
        case home:
            if(hook)
            {
                ROS_MAGENTA_STREAM("Task: return hook");
                task.call_robot_time();
                task.get_hook_new(true);
            }
            if(probe)
            {
                ROS_MAGENTA_STREAM("Task: return probe");
                task.call_robot_time();
                task.return_probe();
            }
            task.config = 2;
            ROS_MAGENTA_STREAM("Task: go home");
            task.call_robot_time();
            task.home();
            task.call_trajectory();
            counter++;
            break;
        default:
            break;
        }

    }while(counter <= sequenz.size()-1 &&ros::ok());

    task.call_robot_time();
    /** ENABLE THIS WHEN LOGGING TO FILE **/
    // task.call_log(false," ");

    auto ende  = std::chrono::high_resolution_clock::now();
    std::cout << "----------------------------" << std::endl;
    std::cout << std::endl << "Zeit: " << std::chrono::duration_cast<std::chrono::seconds>(ende-start).count() << " s" << std::endl;
    std::cout << "----------------------------" << std::endl;
}
