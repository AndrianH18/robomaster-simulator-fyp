#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <string>

std::string groundTruthTopic;
std::string frame_id;
std::string child_frame_id;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = frame_id;
  transformStamped.child_frame_id = child_frame_id;
  transformStamped.transform.translation.x = msg->pose.pose.position.x;
  transformStamped.transform.translation.y = msg->pose.pose.position.y;
  transformStamped.transform.translation.z = msg->pose.pose.position.z;
  transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
  transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
  transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
  transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");

  ros::NodeHandle private_node("~");
  if (! private_node.hasParam("groundTruthTopic"))
  {
    ROS_ERROR("groundTruthTopic param not provided");
    return -1;
  }
  else
  {
    private_node.getParam("groundTruthTopic", groundTruthTopic);
  }
  
  private_node.param<std::string>("frame_id", frame_id, "map");
  private_node.param<std::string>("child_frame_id", child_frame_id, "ground_truth");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(groundTruthTopic, 10, &poseCallback);

  ros::spin();
  return 0;
};