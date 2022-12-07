#include "eigen3/Eigen/Eigen"
#include "nav_msgs/Odometry.h"
#include <ros/package.h>
#include <ros/ros.h>

using namespace std;
Eigen::Matrix4f odometry_to_eigen(nav_msgs::Odometry odom) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = Eigen::Quaternionf(odom.pose.pose.orientation.w,
                                                   odom.pose.pose.orientation.x,
                                                   odom.pose.pose.orientation.y,
                                                   odom.pose.pose.orientation.z)
                                    .toRotationMatrix();
  transform.block<3, 1>(0, 3) =
      Eigen::Vector3f(odom.pose.pose.position.x, odom.pose.pose.position.y,
                      odom.pose.pose.position.z);
  return transform;
}

nav_msgs::Odometry eigen_to_odometry(Eigen::Matrix4f eigen) {
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp = ros::Time::now();
  odom.pose.pose.position.x = eigen(0, 3);
  odom.pose.pose.position.y = eigen(1, 3);
  odom.pose.pose.position.z = eigen(2, 3);
  Eigen::Quaternionf q(eigen.block<3, 3>(0, 0));
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  return odom;
}

ros::Publisher Twt_pub, Twd_pub;
nav_msgs::Odometry mocap_body, mocap_tag;
void mocap_body_callback(nav_msgs::Odometry::ConstPtr msg) {
  mocap_body = *msg;
}
void mocap_tag_callback(nav_msgs::Odometry::ConstPtr msg) { mocap_tag = *msg; }
void color_tag_callback(nav_msgs::Odometry::ConstPtr msg) {
  Eigen::Matrix4f T_color_tag = odometry_to_eigen(*msg);
  Eigen::Matrix4f T_mocap_body = odometry_to_eigen(mocap_body);
  Eigen::Matrix4f T_mocap_tag = odometry_to_eigen(mocap_tag);
  cout << "T_cam_tag: " << endl << T_color_tag << endl;
  cout << "T_mocap_body: " << endl << T_mocap_body << endl;
  cout << "T_mocap_tag: " << endl << T_mocap_tag << endl;
  auto T_body_color =
      T_mocap_body.inverse() * T_mocap_tag * T_color_tag.inverse();
  cout << "T_body_cam: " << endl << T_body_color << endl;
  auto T_mocap_tag_2 = T_mocap_body * T_body_color * T_color_tag;
  cout << "T_mocap_tag_2: " << endl << T_mocap_tag_2 << endl;
  Twt_pub.publish(eigen_to_odometry(T_mocap_tag_2));

  Eigen::Matrix4f T_color_depth = Eigen::Matrix4f::Identity();
  T_color_depth << 0.999987, -0.00484779, -0.00161001, -0.0320387, 0.00498971,
      0.994493, 0.104688, -0.00202164, 0.00109364, -0.104695, 0.994504,
      0.00390838, 0, 0, 0, 1;
  cout << "T_color_depth: " << endl << T_color_depth << endl;
  auto T_body_depth = T_body_color * T_color_depth;
  cout << "T_body_depth: " << endl << T_body_depth << endl;
  auto T_mocap_depth = T_mocap_body * T_body_depth;
  Twd_pub.publish(eigen_to_odometry(T_mocap_depth));
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "republish");
  ros::NodeHandle n("~");
  ros::Rate r(2);
  Twt_pub = n.advertise<nav_msgs::Odometry>("Twt", 1);
  Twd_pub = n.advertise<nav_msgs::Odometry>("Twd", 1);

  ros::Subscriber mocap_cam_sub =
      n.subscribe("/optitrack/Odometry_Kinect", 1000, mocap_body_callback);
  ros::Subscriber mocap_tag_sub =
      n.subscribe("/optitrack/Odometry_Tag", 1000, mocap_tag_callback);
  ros::Subscriber cam_tag_sub =
      n.subscribe("/icar1/agent_localization_node/vision_localization", 1000,
                  color_tag_callback);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}