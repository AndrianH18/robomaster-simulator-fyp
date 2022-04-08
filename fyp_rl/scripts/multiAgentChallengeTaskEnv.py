from openai_ros import multi_robot_gazebo_env
import robot_instance
from gym.envs.registration import register
from gym import spaces
import rospy
import numpy as np
import random
import tf
from std_msgs.msg import String

timestep_limit_per_episode = 3000

register(
    id='ParticleShooterEnv-v1',
    entry_point='multiAgentChallengeTaskEnv:MultiAgentAiChallengeEnv',
    max_episode_steps = timestep_limit_per_episode,
)

class MultiAgentAiChallengeEnv(multi_robot_gazebo_env.MultiRobotGazeboEnv):
    def __init__(self, **kwargs):
        self.robot0 = robot_instance.AiRobot(robot_ns="jackal0",
                                             init_x=rospy.get_param('/train_rl/jackal0/x'),
                                             init_y=rospy.get_param('/train_rl/jackal0/y'),
                                             init_yaw=rospy.get_param('/train_rl/jackal0/yaw'))

        self.robot1 = robot_instance.AiRobot(robot_ns="jackal1",
                                             init_x=rospy.get_param('/train_rl/jackal1/x'),
                                             init_y=rospy.get_param('/train_rl/jackal1/y'),
                                             init_yaw=rospy.get_param('/train_rl/jackal1/yaw'))

        self.hit_detector_sub = rospy.Subscriber("/jackal1/bumper_hit", String, self.hit_callback, queue_size=10)
        self.hit_info_pub = rospy.Publisher("/training/hit_info", String, queue_size=10)

        # Action space: continuous turret angle (x, y). Normalized to [-1, 1].
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        
        # Observations:
        # - Coordinate of target in (x, y), no need for target height (z) as it is constant. This (x, y) coordinate is w.r.t. to
        #   the agent's 'base_link', i.e. tf from agent base_link to target.
        # - Speed of target in (x, y)
        # - [NORMALIZED] Last action (current turret angles before applying a new action)
        # - Whether target was hit
        # - 3D coordinate difference between bullet and target (x, y, z).
        #   This (x, y, z) coordinate is w.r.t. to 'bullet_link' frame, i.e. the tf from bullet to target. The bullet frame has
        #   0 orientation w.r.t. 'map' frame. This means that z is NEGATIVE when the bullet is higher than the target, y is positive
        #   when the bullet is to the RIGHT of the target.
        
        # Bounds: [target_coord_x, target_coord_y, target_speed_x, target_speed_y, last_action_x, last_action_y, is_hit,
        #          bullet_dist_from_target_x, bullet_dist_from_target_y, bullet_dist_from_target_z]
        self.obs_lower_bounds = [ 0.0, -4.0, -1.0, -1.0, -1.0, -1.0, 0, -7.0, -6.0, -2.0]
        self.obs_upper_bounds = [ 8.0,  4.0,  1.0,  1.0,  1.0,  1.0, 1,  7.0,  6.0,  0.1]
        self.observation_space = spaces.Box(np.array(self.obs_lower_bounds), np.array(self.obs_upper_bounds))

        self.tf_listener = tf.TransformListener()
        self.target_health = 20
        self.health_is_decremented = False
        self.agent_coord = [None, None]  # [x, y], the spawn coordinate of the agent robot for the current episode
        self.target_coord = [0, 0]  # [x, y]
        self.target_speed = [0, 0]  # [x, y]
        self.last_action = [0, 0]   # [turret_angle_x, turret_angle_y]
        self.target_is_hit = False
        self.bullet_dist_from_target = [0, 0, 0]   # [x, y, z]

        # Randomization of agent spawn position
        self.randomize_agent_pos = rospy.get_param('/train_rl/jackal0/randomize_pos', default=False)
        if self.randomize_agent_pos:
            self.rand_x_bounds = rospy.get_param('/train_rl/jackal0/rand_x_bounds')
            self.rand_y_bounds = rospy.get_param('/train_rl/jackal0/rand_y_bounds')
        
        # Target movement
        self.move_target = rospy.get_param('/train_rl/jackal1/move', default=False)
        if self.move_target:
            self.move_target_vx_limit = rospy.get_param('/train_rl/jackal1/vx_limit')
            self.move_target_vy_limit = rospy.get_param('/train_rl/jackal1/vy_limit')
        
        random.seed()
        super(MultiAgentAiChallengeEnv, self).__init__(start_init_physics_parameters=True)


    def _set_init_gazebo_pose(self):
        """Sets the Robot in its init pose in Gazebo
        """
        self.robot0._set_init_gazebo_pose(x=self.agent_coord[0], y=self.agent_coord[1])

        vel_x = random.choice([-1, 0, 1])*self.move_target_vx_limit
        vel_y = random.choice([-1, 0, 1])*self.move_target_vy_limit
        self.robot1._set_init_gazebo_pose(vx=vel_x, vy=vel_y)
        self.target_speed = [vel_x, vel_y]

    def _set_init_ros(self):
        """This method is called everytime the environment is reset (i.e. at the start of a new episode).
        """
        self.robot0._set_init_ros()
        self.robot1._set_init_ros()

        # Get a new randomized spawn coordinate for the agent robot.
        self.agent_coord[0] = random.uniform(*self.rand_x_bounds) if self.randomize_agent_pos else None
        self.agent_coord[1] = random.uniform(*self.rand_y_bounds) if self.randomize_agent_pos else None

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self.robot0._check_all_systems_ready()
        self.robot1._check_all_systems_ready()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        self.target_health = 20
        self.health_is_decremented = False
        self.last_action = [0, 0]
        self.target_is_hit = False
        self.bullet_dist_from_target = [0, 0, 0]

    def _get_obs(self):
        """
        Here we define what sensor data of our robots observations
        To know which Variables we have access to, we need to read the
        MyRobotEnv API DOCS
        :return: observations
        """
        try:
            # Transform from agent base_link to target (the argument order is flipped!).
            trans, _ = self.tf_listener.lookupTransform(target_frame="jackal0/ground_truth", source_frame="jackal1/ground_truth/front_armor_sensor",
                                                        time=rospy.Time(0))
            self.target_coord = [trans[0], trans[1]]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
            rospy.logerr_throttle(0.2, f"ROS tf exception occured in MultiAgentAiChallengeEnv._get_obs(): {err}")

        observations = [self.target_coord[0], self.target_coord[1], self.target_speed[0], self.target_speed[1], self.last_action[0],
                        self.last_action[1], float(self.target_is_hit), self.bullet_dist_from_target[0], self.bullet_dist_from_target[1],
                        self.bullet_dist_from_target[2]]
        rospy.logdebug(f"Observation: {observations}")
        return observations

    def _set_action(self, action):
        """
        Action for 1 time step: move turret to aim, shoot, see if bullet hits.

        Action space is normalized to [-1, 1]. Turret's actual angle range is x: [-1.57, 1.57], y: [-0.5, 0.8].
        Need to scale the action to cover the turret's range or to narrow turret's action range.
        """
        # Note: target coordinate and speed don't need to be updated here, only update them in `_get_obs()`.
        # This is fine because between calling _get_obs() and the next call of _set_action(), the simulation is paused.
        action_x = action[0]*0.9
        action_y = action[1]*0.3
        self.robot0.move_turret(action_x, action_y)
        self.hit_info_pub.publish(String(f"Setting turret to ({action_x}, {action_y})"))  # TODO: these are for debugging, remove!!
        self.robot0.wait_until_arrived(action_x, action_y, pos_tolerance=0.015, speed_tolerance=0.1)
        self.hit_info_pub.publish(String("Shooting!"))  # TODO: these are for debugging, remove!!
        self.target_is_hit = False   # Reset flags just before shooting.
        self.health_is_decremented = False
        self.robot0.shoot()
        self.wait_until_hit_or_miss()

        self.last_action = action
        # Set the positions and speed for both robots at every step instead of just once at the beginning of the episode, so that:
        # (1) the target robot does not hit the arena walls after moving for a while without being teleported back to the center
        # (2) the robots do not drift on their own (due to Gazebo physics) if the episode is long.
        self._set_init_gazebo_pose()
        # Small delay before exiting this method to make sure the required tf in self._get_obs() is updated before pausing the sim.
        rospy.sleep(0.05)
        
    def _is_done(self, observations):
        """
        Decide if episode is done based on the observations
        """
        if self.target_health <= 0:
            return True
        return False

    def _compute_reward(self, observations, done):
        """
        Return the immediate reward for the step based on the observations given
        """
        reward = 0
        miss_penalty_range = [-3, -0.05]  # [heaviest_penalty, lightest_penalty]
        scale_factor = 1.0                # Scaling factor for euclidean distance from bullet to target

        if done:
            reward = 200    # Reward for depleting the target's health.

        if self.target_is_hit:
            reward += 20    # Reward for hitting the target.
        else:               # Penalty for missing the target. Penalty is a negative number!
            reward += np.clip( -1*scale_factor*np.linalg.norm(self.bullet_dist_from_target), *miss_penalty_range )
        
        # Publish information for the step.
        if self.target_is_hit:
            info_str = f">>>>>>>>>>>>>>>>>> Target hit! Health: {self.target_health}"
            rospy.logdebug(info_str)
            self.hit_info_pub.publish(String(info_str))
        else:
            # TODO: use custom message instead of std_msgs::String.
            self.hit_info_pub.publish(String(f"Missed. Health:{self.target_health}, rew:{reward}, bullet:{self.bullet_dist_from_target}"))
        
        return reward

    def wait_until_hit_or_miss(self, starting_delay=0.05):
        """
        Blocking method that returns when either of the following conditions is met:
         - Bullet hits the target. self.target_is_hit is set to True by self.hit_callback().
         - Bullet misses target (bullet has gone past the target's x coordinate without hitting the target).
         - Bullet hits the ground before hitting the target.
        
        This method also updates self.bullet_dist_from_target, required for the observation.
        """
        # Starting delay is needed because the tf data might be a bit old. If no delay is introduced here, the tf data used for
        # this method may be stale (e.g. tf is still representing bullet position a while ago, e.g. on the ground.), possibly
        # causing this method to immediately and incorrectly return.
        rospy.sleep(starting_delay)
        while not self.target_is_hit:
            try:
                # Transform from bullet to target (the argument order is flipped!).
                trans, _ = self.tf_listener.lookupTransform(target_frame="bullet_link", source_frame="jackal1/ground_truth/front_armor_sensor",
                                                            time=rospy.Time(0))
                bullet_dist_x = trans[0]
                bullet_dist_y = trans[1]
                bullet_dist_z = trans[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
                rospy.logerr_throttle(0.2, f"ROS tf exception occured in MultiAgentAiChallengeEnv.hit_or_miss(): {err}")
                continue
        
            if  bullet_dist_x < -0.05 or bullet_dist_z > 0.05:  # bullet_dist_z ~= 0.09 when on the ground
                # Update self.bullet_dist_from_target.
                self.bullet_dist_from_target = [np.clip(bullet_dist_x, self.obs_lower_bounds[-3], self.obs_upper_bounds[-3]),
                                                np.clip(bullet_dist_y, self.obs_lower_bounds[-2], self.obs_upper_bounds[-2]),
                                                np.clip(bullet_dist_z, self.obs_lower_bounds[-1], self.obs_upper_bounds[-1]) ]
                return
        
        # Target is hit, distance between bullet and armor/target is therefore zero.
        self.bullet_dist_from_target = [0, 0, 0]
        return

    def hit_callback(self, msg):
        if msg.data == "front_bumper_vals":
            self.target_is_hit = True

            # Decrement the health here. This is about the fastest possible way so as not to "miss" decrementing the health.
            if not self.health_is_decremented:
                self.target_health -= 1
                self.health_is_decremented = True

# TODO FIXME: fix bugs:
"""
1. Sometimes health is decremented 2 times for 1 hit.
   Is collision detector too sensitive? Need mutex for hit_callback() due to possible 2 threads running hit_callback?
   (due to fast publish in /bumper_hit)
2. Sometimes bullet_dist (x, y, z) is still wrong (stale?), might be due to wait_turret() returning quickly because current action ~= prev action
"""