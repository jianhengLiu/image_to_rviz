/*
 * @Author: Jianheng Liu
 * @Date: 2022-01-17 14:57:48
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-06-16 20:08:01
 * @Description: Description
 */
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

geometry_msgs::PoseStamped init_pose;
nav_msgs::Path             gt_path;
Eigen::Quaterniond         init_q;
ros::Publisher             pub_reset_pos, pub_gt;
bool                       is_init = false;
void                       pose_callback(geometry_msgs::PoseStamped _pose_msg)
{
    if (!is_init)
    {
        init_pose = _pose_msg;
        init_q    = Eigen::Quaterniond(_pose_msg.pose.orientation.w, _pose_msg.pose.orientation.x,
                                       _pose_msg.pose.orientation.y, _pose_msg.pose.orientation.z);
        is_init   = true;
    }
    _pose_msg.pose.position.x -= init_pose.pose.position.x;
    _pose_msg.pose.position.y -= init_pose.pose.position.y;
    _pose_msg.pose.position.z -= init_pose.pose.position.z;
    Eigen::Vector3d p(_pose_msg.pose.position.x, _pose_msg.pose.position.y,
                      _pose_msg.pose.position.z);
    p = init_q.toRotationMatrix().transpose() * p;

    Eigen::Quaterniond q =
        Eigen::Quaterniond(_pose_msg.pose.orientation.w, _pose_msg.pose.orientation.x,
                           _pose_msg.pose.orientation.y, _pose_msg.pose.orientation.z);
    q = init_q.toRotationMatrix().transpose() * q;

    _pose_msg.pose.position.x    = p.x();
    _pose_msg.pose.position.y    = p.y();
    _pose_msg.pose.position.z    = p.z();
    _pose_msg.pose.orientation.w = q.w();
    _pose_msg.pose.orientation.x = q.x();
    _pose_msg.pose.orientation.y = q.y();
    _pose_msg.pose.orientation.z = q.z();
    pub_reset_pos.publish(_pose_msg);

    gt_path.header = _pose_msg.header;
    gt_path.poses.push_back(_pose_msg);
    pub_gt.publish(gt_path);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mav_fsm");
    ros::NodeHandle nh("~");

    ros::Subscriber sub_rc =
        nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody1/pose", 10, pose_callback);

    pub_reset_pos = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/remap_pose", 10);
    pub_gt        = nh.advertise<nav_msgs::Path>("/gt", 10);

    ros::spin();
    return 0;
}
