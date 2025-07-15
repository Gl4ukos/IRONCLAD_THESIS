// pose_to_tf.cpp
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

void poseCallback(const nav_msgs::Odometry msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z));
  tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y , msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, msg.header.stamp , "world", "base_link"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_to_tf");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/pose", 10, poseCallback);

  ros::spin();
  return 0;
}
