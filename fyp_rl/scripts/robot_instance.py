import robot_underlying
# from gym.envs.registration import register
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_srvs.srv import Empty


class AiRobot(robot_underlying.Robot):
    def __init__(self, **kwargs):
        self.robot_name_space = kwargs['robot_ns']
        self.init_x   = kwargs['init_x']
        self.init_y   = kwargs['init_y']
        self.init_yaw = kwargs['init_yaw']
        self.quat = quaternion_from_euler(0, 0, self.init_yaw)
        self.reset_gazebo_simulation_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.reset_localization_proxy = rospy.Publisher('/' + self.robot_name_space + '/initialpose',
                                                        PoseWithCovarianceStamped, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/' + self.robot_name_space + '/cmd_vel', Twist, queue_size=1)
        # self.clean_global_costmap_proxy = rospy.ServiceProxy(
        #     '/' + self.robot_name_space + '/global_costmap/clean_costmap', Empty)
        # self.clean_local_costmap_proxy = rospy.ServiceProxy(
        #     '/' + self.robot_name_space + '/local_costmap/clean_costmap', Empty)
        # self.clean_decision_costmap_proxy = rospy.ServiceProxy('/'+self.robot_name_space+'/decision_costmap/clean_costmap', Empty)

        # Here we will add any init functions prior to starting the MyRobotEnv
        super(AiRobot, self).__init__(self.robot_name_space)


    def _set_init_gazebo_pose(self, x=None, y=None, yaw=None, vx=0, vy=0):
        """Sets the Robot in its init pose in Gazebo
        """
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            robot_pose = ModelState()
            robot_pose.model_name = self.robot_name_space
            robot_pose.reference_frame = "/map"
            robot_pose.pose.position.x = self.init_x if x==None else x
            robot_pose.pose.position.y = self.init_y if y==None else y
            robot_pose.pose.position.z = 0
            robot_pose.pose.orientation.x = self.quat[0]
            robot_pose.pose.orientation.y = self.quat[1]
            robot_pose.pose.orientation.z = self.quat[2]
            robot_pose.pose.orientation.w = self.quat[3]
            self.reset_gazebo_simulation_proxy(robot_pose)
            
            cmd_msg = Twist()
            cmd_msg.linear.x = vx
            cmd_msg.linear.y = vy
            self.cmd_vel_pub.publish(cmd_msg)
            return True
            
        except rospy.ServiceException as e:
            print ("/gazebo/reset_simulation service call failed")
            return False

    def _set_init_ros(self, x=None, y=None, yaw=None):
        # self.move_turret(0)
        init_msg = PoseWithCovarianceStamped()
        init_msg.header.stamp = rospy.Time.now()
        init_msg.header.frame_id = 'map'
        init_msg.pose.pose.position.x = self.init_x if x==None else x
        init_msg.pose.pose.position.y = self.init_y if y==None else y
        init_msg.pose.pose.orientation.x = self.quat[0]
        init_msg.pose.pose.orientation.y = self.quat[1]
        init_msg.pose.pose.orientation.z = self.quat[2]
        init_msg.pose.pose.orientation.w = self.quat[3]
        # rospy.wait_for_service('/' + self.robot_name_space + '/global_costmap/clean_costmap')
        # The first reset will have problems, you need to sleep for a while and wait for it to respond
        self.reset_localization_proxy.publish(init_msg)
        rospy.sleep(0.1)
        # Clean up the map
        # try:
        #     self.clean_global_costmap_proxy()
        # except rospy.ServiceException as e:
        #     print('/' + self.robot_name_space + '/global_costmap/clean_costmap', " service call failed")
        # rospy.wait_for_service('/' + self.robot_name_space + '/local_costmap/clean_costmap')
        # try:
        #     self.clean_local_costmap_proxy()
        # except rospy.ServiceException as e:
        #     print('/' + self.robot_name_space + '/local_costmap/clean_costmap', " service call failed")
        # # self.clean_decision_costmap_proxy()
        # rospy.sleep(0.1)
        return True

    def _init_env_variables(self, **kwargs):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        """Get observations.
        """
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()
    
    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()
