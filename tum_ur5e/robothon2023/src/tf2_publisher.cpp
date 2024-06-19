/**
 * @file tf2_publisher.cpp
 * @authors Adrian Müller (adrian.mueller@study.thws.de), 
 *          Maximilian Hornauer (maximilian.hornauer@study.thws.de),
 *          Usama Ali (usama.ali@study.thws.de)
 * @brief Program to publish stored points
 * @version 0.1
 * @date 2023-03-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include <ros/ros.h>
#include <ros/package.h>

#include <fstream>
#include <robothon2023/eigen_json.hpp>

#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>
#include <robothon2023/AddTf2.h>

#include <visualization_msgs/Marker.h>

using json = nlohmann::json;



class tf2_publisher
{
private:
    ros::ServiceServer server_tf_;
    ros::Publisher vis_pub_;
    ros::AsyncSpinner* spinner;
    std::string filename;
    std::map<std::string, geometry_msgs::TransformStamped> transforms;
    tf2_ros::StaticTransformBroadcaster* br;
    bool add_tf(robothon2023::AddTf2::Request& req,
                robothon2023::AddTf2::Response& res);
    void writeJson();
    void readJson();
    void publishTf();
    void visual_taskboard();

public:
    tf2_publisher(ros::NodeHandle& n, std::string filename_);
    ~tf2_publisher();

};

tf2_publisher::tf2_publisher(ros::NodeHandle& n, std::string filename_)
{
    this->server_tf_ = n.advertiseService("/store_tf",&tf2_publisher::add_tf, this);
    this->vis_pub_ = n.advertise<visualization_msgs::Marker>("/visualization_marker", 0 ,true);
    
    this->spinner = new ros::AsyncSpinner(2);
    this->br = new tf2_ros::StaticTransformBroadcaster();
    this->filename = filename_;

    this->readJson();
    this->publishTf();
}

tf2_publisher::~tf2_publisher()
{
    this->writeJson();
}

void tf2_publisher::visual_taskboard()
{
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "task_board";
    marker.header.stamp = ros::Time();
    marker.ns = "task_board";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.133;//0.1265;
    marker.pose.position.y = 0.085;//0.061;0.076
    marker.pose.position.z = -0.061 - 0.009;//-0.039; //9mm offset because tcp is broken #FIXME
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://robothon2023/config/Taskboard.dae";
    marker.mesh_use_embedded_materials = true;
    vis_pub_.publish(marker);
}

void tf2_publisher::publishTf() {
    for(auto i : this->transforms) {
        this->br->sendTransform(i.second);
    }
    this->visual_taskboard();
}

void tf2_publisher::readJson() {
    std::string path = ros::package::getPath("robothon2023");

    std::ifstream f(path+"/config/"+this->filename);
    json j = json::parse(f);

    for(auto element : j.items())
    {   
        std::cout << "Importing " << element.key() << std::endl;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = element.value()["parent"];
        transformStamped.child_frame_id = element.value()["name"];
        Eigen::Matrix4d mat= element.value()["transform"];
        tf::transformEigenToMsg(Eigen::Isometry3d(mat),transformStamped.transform);
        transforms[element.value()["name"]] = transformStamped;
    }
}

void tf2_publisher::writeJson()
{   
    std::cout << "Writing file..." << std::endl;
    json j;

    for(auto i : this->transforms) {
        geometry_msgs::TransformStamped transf = i.second;
        Eigen::Isometry3d iso;
        tf::transformMsgToEigen(transf.transform, iso);
        j[i.first]["parent"] = transf.header.frame_id;
        j[i.first]["name"] = transf.child_frame_id;
        j[i.first]["transform"] = iso.matrix();
    }

    std::string path = ros::package::getPath("robothon2023");
    std::ofstream file(path+"/config/"+this->filename);
    file << std::setw(4) << j;
}
/**
 * @brief Service call to add a tf to the transforms buffer
 * 
 * @param req the request with the transform and the information if the pose should be recalculated in relation to the taskboard
 * @param res the response
 * @return true 
 * @return false 
 */
bool tf2_publisher::add_tf(robothon2023::AddTf2::Request& req,
                            robothon2023::AddTf2::Response& res)
{
    ROS_INFO_STREAM("Callback called with " << req);
    geometry_msgs::TransformStamped temp = req.pose;
    /* The pose needs to be retransformed because it is given in relation to the base_link but we want it in relation to the taskboard*/
    if(req.relative) {
        Eigen::Isometry3d iso;
        tf::transformMsgToEigen(temp.transform,iso);
        Eigen::Matrix4d mat_f = iso.matrix();

        auto tb = transforms["task_board"]; // get taskboard to base_link
        tf::transformMsgToEigen(tb.transform,iso);
        Eigen::Matrix4d mat_tb = iso.matrix();

        Eigen::Matrix4d mat_rel = mat_tb.inverse() * mat_f; //calculate target in reference to task_board
        tf::transformEigenToMsg(Eigen::Isometry3d(mat_rel),temp.transform);
    }
    else{
        temp = req.pose; // we dont want to change anything in the transform
    }

    transforms[temp.child_frame_id] = temp; // write the transform into buffer
    this->publishTf(); // republish everything
    res.success=true;
    return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tf2_publisher");
    ros::NodeHandle n;

    std::string filename;

    if (n.getParam("json_filename", filename))
    {
        std::cout << "filename: " << filename << std::endl;
    }
    else
    {
        std::cout << "FAILED " << std::endl;
        std::cout << "filename: " << filename << std::endl;
        return 0;
    }
    
    tf2_publisher temp(n, filename);
    
    ros::spin();
    
    return 0;
}

