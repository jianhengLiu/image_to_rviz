/*
 * @Author: Jianheng Liu
 * @Date: 2022-05-22 11:16:58
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-06-16 10:31:24
 * @Description: Description
 */
#include "eigen3/Eigen/Eigen"
#include "nav_msgs/Path.h"
#include <dynamic_reconfigure/server.h>
#include <eigen3/Eigen/src/Geometry/Transform.h>
#include <image_to_rviz/paramsConfig.h>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

double lu_x = 0, lu_y = 0;
double yaw = -3.14;
double path_yaw = -3.14;
double sizeSq = 1;
double z = -10;

void callback(image_to_rviz::paramsConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %lf %lf %lf %lf %lf", config.size_param, config.dx_param, config.dy_param,
             config.yaw, config.z);

    lu_x = config.dx_param;
    lu_y = config.dy_param;
    yaw = config.yaw;
    path_yaw = config.path_yaw;
    sizeSq = config.size_param;
    z = config.z;
};

ros::Publisher marker_pub;
void pathCallback(nav_msgs::Path::ConstPtr msg)
{
    nav_msgs::Path republish_path = *msg;
    for (auto &pose : republish_path.poses)
    {
        Eigen::Vector3d pose_t(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        Eigen::Isometry3d transform;
        transform.setIdentity();
        transform.rotate(Eigen::AngleAxisd(path_yaw, Eigen::Vector3d::UnitZ()));

        pose_t = transform * pose_t;
        pose.pose.position.x = pose_t(0);
        pose.pose.position.y = pose_t(1);
        pose.pose.position.z = pose_t(2);
    }
    marker_pub.publish(republish_path);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "republish");
    ros::NodeHandle n("~");
    ros::Rate r(2);
    marker_pub = n.advertise<nav_msgs::Path>("path_republish", 1);

    ros::Subscriber path_sub = n.subscribe("/vins_estimator/path_rgbd", 1000, pathCallback);

    dynamic_reconfigure::Server<image_to_rviz::paramsConfig> server;
    dynamic_reconfigure::Server<image_to_rviz::paramsConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}
