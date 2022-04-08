#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>
#include <math.h>
#include <std_msgs/String.h>

class CollisionDetector
{
    public:
        CollisionDetector(ros::NodeHandle& nodehandle) :
        nh_(nodehandle)
        {
            force_threshold_ = nh_.param("force_threshold", 1e-250);
            bumperPublisher_ = nh_.advertise<std_msgs::String>("bumper_hit", 1, false);
            bumperTopics_ = {"front_bumper_vals", "back_bumper_vals", "left_bumper_vals", "right_bumper_vals"};

            for (int i = 0; i < bumperSubscribers_.size(); i++) {
                bumperSubscribers_[i] = nh_.subscribe<gazebo_msgs::ContactsState>(bumperTopics_[i], 1,
                    std::bind(&CollisionDetector::generalBumperCallback_, this, std::placeholders::_1, bumperTopics_[i]));
            }
        }

    private:
        void generalBumperCallback_(const gazebo_msgs::ContactsState::ConstPtr &msg, std::string bumperSide)
            {
                if (!msg->states.empty()){
                    auto x = msg->states[0].wrenches[0].force.x;
                    auto y = msg->states[0].wrenches[0].force.y;
                    
                    if (abs(x) >= force_threshold_ || abs(y) >= force_threshold_){
                        ROS_INFO_STREAM("Hit detected on bumper side: " << bumperSide);
                        std_msgs::String pub_msg;
                        pub_msg.data = bumperSide;
                        bumperPublisher_.publish(pub_msg);  // Publish a character code representing the bumper side
                    }
                }
            }

        ros::NodeHandle                 nh_;
        ros::Publisher                  bumperPublisher_;
        std::array<ros::Subscriber, 4>  bumperSubscribers_;
        std::array<std::string, 4>      bumperTopics_;
        double                          force_threshold_;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "smb_highlevel_controller");
    ros::NodeHandle nodeHandle("~");
    CollisionDetector collisionDetector(nodeHandle);
    ros::Rate loop_rate(120);
    while (ros::ok()){
        loop_rate.sleep();
        ros::spinOnce();
     }   
    return 0;
}