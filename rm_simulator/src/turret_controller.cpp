#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/time.h>

class TurretPositionController
{
public:
    TurretPositionController(ros::NodeHandle& nodehandle) :
    nh_(nodehandle)    
    {        
        turret_yaw_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("turret_yaw_controller/command", 1, true);
        turret_pitch_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("turret_pitch_controller/command", 1, true);
        turret_position_sub = nh_.subscribe("turret_position", 10, &TurretPositionController::positionCallback, this);
        
        yaw_traj.header.frame_id = "front_camera_mount";
        yaw_traj.joint_names.resize(1);
        yaw_traj.points.resize(1);
        yaw_traj.points[0].positions.resize(1);
        yaw_traj.points[0].effort.resize(1);
        yaw_traj.joint_names[0] ="front_camera_pivot_joint";

        pitch_traj.header.frame_id = "front_camera_beam";
        pitch_traj.joint_names.resize(1);
        pitch_traj.points.resize(1);
        pitch_traj.points[0].positions.resize(1);
        pitch_traj.points[0].effort.resize(1);
        pitch_traj.joint_names[0] ="front_turret_joint";
    }

    void positionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
        yaw_traj.header.stamp = ros::Time::now();
        yaw_traj.points[0].positions[0] = msg->data[0];
        yaw_traj.points[0].effort[0] = 100;        
        yaw_traj.points[0].time_from_start = ros::Duration(0.05);
        turret_yaw_pub.publish(yaw_traj);

        pitch_traj.header.stamp = ros::Time::now();
        pitch_traj.points[0].positions[0] = msg->data[1];
        pitch_traj.points[0].effort[0] = 100;        
        pitch_traj.points[0].time_from_start = ros::Duration(0.05);
        turret_pitch_pub.publish(pitch_traj);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher  turret_yaw_pub;
    ros::Publisher  turret_pitch_pub;
    ros::Subscriber turret_position_sub;
    trajectory_msgs::JointTrajectory yaw_traj;
    trajectory_msgs::JointTrajectory pitch_traj;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "turret_position_controller_node");
    ros::NodeHandle node_handle;
    // ros::AsyncSpinner spinner(1);
    // spinner.start();

    TurretPositionController controller(node_handle);

    ros::spin();
    return 0;
}
