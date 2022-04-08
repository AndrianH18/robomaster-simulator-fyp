#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>


class BulletTfBroadcaster {
    public:
        BulletTfBroadcaster(std::string child_frame_id):
            nh_(ros::NodeHandle()),
            parent_frame_id_("map"),
            child_frame_id_(child_frame_id)
        {
            ros::Subscriber sub_ = nh_.subscribe("gazebo/model_states", 1, &BulletTfBroadcaster::modelStateCallback, this);
            transform_.setRotation(tf::Quaternion(0, 0, 0, 1));
            ros::spin();
        }

    private:
        void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
            transform_.setOrigin(tf::Vector3(msg->pose[0].position.x, msg->pose[0].position.y, msg->pose[0].position.z));
            br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), parent_frame_id_, child_frame_id_));
        }
        
        ros::NodeHandle nh_;
        std::string parent_frame_id_;
        std::string child_frame_id_;
        tf::TransformBroadcaster br_;
        tf::Transform transform_;
        ros::Subscriber sub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "bullet_tf_broadcaster");
  BulletTfBroadcaster bullet_tf_broadcaster("bullet_link");
  
  return 0;
}
