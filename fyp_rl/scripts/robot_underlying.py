#! /usr/bin/env python

from ntpath import join
import rospy
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import numpy as np

class Robot():
    """Superclass for all CubeSingleDisk environments.
    """

    def __init__(self, robot_ns):
        """Initializes a new CubeSingleDisk environment.
        """
        self.robot_name_space = robot_ns

        """
        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that the stream of data doesn't flow. This is for simulations
        that are pause for whatever reason
        2) If the simulation was running already for some reason, we need to reset the controllers.
        This has to do with the fact that some plugins with tf don't understand the reset of the simulation
        and need to be reset to work properly.
        """
        self._check_all_sensors_ready()

        # We Start all the ROS related Subscribers and publishers
        rospy.Subscriber('/'+self.robot_name_space+'/amcl_pose', PoseStamped, self._pose_callback)
        rospy.Subscriber('/'+self.robot_name_space+'/odometry/filtered', Odometry, self._odom_callback)

        self._turret_position_pub = rospy.Publisher('/'+self.robot_name_space+'/turret_position',
                                        Float32MultiArray, queue_size=1)
        self._shoot_cmd_pub = rospy.Publisher('/shoot', String, queue_size=1)
        self._check_publishers_connection()

    def _pose_callback(self, data):
        self.amcl_pose = data

    def _odom_callback(self, data):
        self.odom = data

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        self._check_publishers_connection()
        return True

    def _check_all_sensors_ready(self):
        self._check_joint_states_ready()
        self._check_amcl_pose_ready()
        self._check_odom_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_joint_states_ready(self):
        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message('/'+self.robot_name_space+'/joint_states', JointState, timeout=1.0)
                rospy.logdebug("Current " + '/'+self.robot_name_space+'/joint_states'+ " READY")
            except:
                rospy.logerr("Current " + '/'+self.robot_name_space+'/joint_states'+ " not ready yet, retrying for getting joint_states")
        return self.joints

    def _check_amcl_pose_ready(self):
        self.amcl_pose = None
        while self.amcl_pose is None and not rospy.is_shutdown():
            try:
                self.amcl_pose = rospy.wait_for_message('/'+self.robot_name_space+'/amcl_pose', PoseStamped, timeout=1.0)
                rospy.logdebug("Current " + '/' + self.robot_name_space + '/amcl_pose' + " READY")
            except:
                rospy.logdebug("Current " + '/' + self.robot_name_space + '/amcl_pose' + " not ready yet, retrying for getting amcl")
        return self.amcl_pose

    def _check_odom_ready(self):
        self.odom = None
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message('/'+self.robot_name_space+'/odometry/filtered', Odometry, timeout=1.0)
                rospy.logdebug("Current " + '/' + self.robot_name_space + '/odometry/filtered' + " READY")
            except:
                rospy.logdebug("Current " + '/' + self.robot_name_space + '/odometry/filtered' + " not ready yet, retrying for getting odom")
        return self.odom

    def _check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        pass
        # rate = rospy.Rate(10)  # 10hz
        # while self._turret_position_pub.get_num_connections() == 0 and not rospy.is_shutdown():
        #     rospy.logdebug("No susbribers to _turret_position_pub yet so we wait and try again")
        #     try:
        #         rate.sleep()
        #     except rospy.ROSInterruptException:
        #         # This is to avoid error when world is rested, time when backwards.
        #         pass
        # rospy.logdebug("_turret_position_pub Publisher Connected")
        #
        # while self._goal_pub.get_num_connections() == 0 and not rospy.is_shutdown():
        #     rospy.logdebug("No susbribers to _goal_pub yet so we wait and try again")
        #     try:
        #         rate.sleep()
        #     except rospy.ROSInterruptException:
        #         # This is to avoid error when world is rested, time when backwards.
        #         pass
        # rospy.logdebug("_goal_pub Publisher Connected")
        #
        # rospy.logdebug("All Publishers READY")


    # Methods that the TaskEnvironment will need.
    # ----------------------------
    def move_turret(self, target_x, target_y):
        turret_position = Float32MultiArray()
        turret_position.data = [target_x, target_y]
        self._turret_position_pub.publish(turret_position)

    def wait_until_arrived(self, target_x, target_y, pos_tolerance=0.01, speed_tolerance=0.07, starting_delay=0.05):
        # 0.01 rad = 0.573 degrees
        rospy.sleep(starting_delay)
        rate = rospy.Rate(30)
        start_wait_time = rospy.get_rostime().to_sec()
        end_wait_time = 0.0
        while not rospy.is_shutdown():
            joint_data = self._check_joint_states_ready()
            if len(joint_data.name) == 1:
                continue    # Published joint_data alternates between 'front_rocker' and turret/wheel joints. We don't want 'front_rocker'.  
            current_x = joint_data.position[0]
            current_y = joint_data.position[3]
            speed_x = joint_data.velocity[0]
            speed_y = joint_data.velocity[3]
            # rospy.logdebug("POSITION=" + str([current_x, current_y]))
            if np.linalg.norm([target_x-current_x, target_y-current_y]) < pos_tolerance and speed_x < speed_tolerance and speed_y < speed_tolerance:
                rospy.logdebug(f"Turret reached desired position [{target_x}, {target_y}]!")
                end_wait_time = rospy.get_rostime().to_sec()
                break
            rate.sleep()
        delta_time = end_wait_time - start_wait_time
        rospy.logdebug("Turret wait time=" + str(delta_time))
        return delta_time

    def shoot(self):
        robot_index = String(self.robot_name_space[-1])
        self._shoot_cmd_pub.publish(robot_index)

    def get_amcl_pose(self):
        return self.amcl_pose


    # Methods that the TaskEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TaskEnvironment.
    # ----------------------------
    def _set_init_gazebo_pose(self):
        """Sets the Robot in its init pose in Gazebo
        """
        raise NotImplementedError()

    def _set_init_ros(self):
        """Sets the Robot in its init pose in ROS
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
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