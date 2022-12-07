/*
 * @Author: Jianheng Liu
 * @Date: 2022-05-22 11:16:58
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-05-26 02:54:02
 * @Description: Description
 */
#include "eigen3/Eigen/Eigen"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <dynamic_reconfigure/server.h>
#include <image_to_rviz/paramsConfig.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/Marker.h>

double lu_x = 0, lu_y = 0;
double yaw = -3.14;
double path_yaw = -3.14;
double sizeSq = 1;
double z = -10;

double llu_x = 0, llu_y = 0;
double lyaw = -3.14;
double lpath_yaw = -3.14;
double lsizeSq = 1;
double lz = -10;
bool is_first = true;

// tf2_msgs/TFMessage
ros::Publisher marker_pub;
nav_msgs::Path republish_path;
std::vector<nav_msgs::Path> paths;
void callback(image_to_rviz::paramsConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %lf %lf %lf %lf %lf", config.size_param, config.dx_param, config.dy_param,
             config.yaw, config.z);

    llu_x = lu_x;
    llu_y = lu_y;
    lyaw = yaw;
    lpath_yaw = path_yaw;
    lsizeSq = sizeSq;
    lz = z;

    lu_x = config.dx_param;
    lu_y = config.dy_param;
    yaw = config.yaw;
    path_yaw = config.path_yaw;
    sizeSq = config.size_param;
    z = config.z;
};
Eigen::Isometry3d transform_init;
void pathCallback(tf2_msgs::TFMessage msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header = msg.transforms.back().header;
    pose.header.frame_id = "map";
    pose.pose.position.x = msg.transforms.back().transform.translation.x;
    pose.pose.position.y = msg.transforms.back().transform.translation.y;
    pose.pose.position.z = msg.transforms.back().transform.translation.z;
    pose.pose.orientation.x = msg.transforms.back().transform.rotation.x;
    pose.pose.orientation.y = msg.transforms.back().transform.rotation.y;
    pose.pose.orientation.z = msg.transforms.back().transform.rotation.z;
    pose.pose.orientation.w = msg.transforms.back().transform.rotation.w;
    republish_path.poses.push_back(pose);

    if (is_first)
    {
        Eigen::Vector3d pose_t(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        transform_init.setIdentity();
        transform_init.translate(pose_t);
        transform_init.rotate(Eigen::Quaterniond(pose.pose.orientation.w, pose.pose.orientation.x,
                                                 pose.pose.orientation.y, pose.pose.orientation.z));
        is_first = false;
    }

    republish_path.header = msg.transforms.back().header;
    republish_path.header.frame_id = "map";
    paths.push_back(republish_path);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "republish_gt");
    ros::NodeHandle n("~");
    ros::Rate r(2);
    marker_pub = n.advertise<nav_msgs::Path>("gt_path", 1);

    ros::Subscriber path_sub = n.subscribe("/gt", 1000, pathCallback);

    dynamic_reconfigure::Server<image_to_rviz::paramsConfig> server;
    dynamic_reconfigure::Server<image_to_rviz::paramsConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    while (ros::ok())
    {
        if (!paths.empty())
        {
            nav_msgs::Path path = paths.back();
            for (auto &pose : path.poses)
            {
                Eigen::Vector3d pose_t(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                Eigen::Isometry3d transform;
                transform.setIdentity();
                transform.rotate(Eigen::AngleAxisd(path_yaw, Eigen::Vector3d::UnitZ()));

                pose_t = transform * transform_init.inverse() * pose_t;
                pose.pose.position.x = pose_t(0) + lu_x;
                pose.pose.position.y = pose_t(1) + lu_y;
                pose.pose.position.z = pose_t(2);
            }
            marker_pub.publish(path);
            paths.pop_back();
        }
        ros::spinOnce();
        r.sleep();
    }
}
