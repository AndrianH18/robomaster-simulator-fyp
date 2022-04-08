#!/usr/bin/env python
import multiAgentChallengeTaskEnv
import gym
from stable_baselines3 import PPO
import os
import numpy as np
import rospy
import rospkg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

MODEL_NAME    = 'PPO_stage1_1'
MODEL_TO_LOAD = '490000_steps.zip'
# Agent coordinates (in Gazebo map frame) to evaluate
AGENT_COORDS_X = [4.00, 3.20, 2.40, 1.60]
AGENT_COORDS_Y = [3.65, 2.95, 2.25, 1.55, 0.85]
NUM_EPISODES = 10


reset_gazebo_simulation_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

def main():
    rospy.init_node('evaluate_particle_shooter_node', anonymous=False, log_level=rospy.DEBUG)

    # Tensorboard logging
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('fyp_rl')
    model_path = os.path.join(pkg_path, 'training_results', MODEL_NAME, MODEL_TO_LOAD)
    save_path = os.path.join(pkg_path, 'eval_results')

    if not os.path.exists(save_path):
        os.makedirs(save_path)

    env = gym.make('ParticleShooterEnv-v1')

    try:
        model = PPO.load(model_path, env, verbose=1)
        rospy.loginfo(f"Loaded model: {model_path}")
    except Exception as e:
        rospy.logerr(f"Exception occurred while loading model: {e}")
        rospy.signal_shutdown("Exception occurred while loading model")
        return

    rospy.loginfo(f"Starting model evaluation...")
    try:
        eval_coords = [(x, y) for x in AGENT_COORDS_X for y in AGENT_COORDS_Y]
        coord_mean_rewards = []
        coord_mean_hit_probs = []
        
        for x, y in eval_coords:
            all_episode_total_rewards = 0
            all_episode_hit_ratio = 0
            
            for _ in range(NUM_EPISODES):
                obs = env.reset()
                set_model_state(x, y)
                step_count = 0
                total_ep_reward = 0
                hit_count = 0
                episode_done = False
                while not episode_done:
                    step_count += 1
                    pred_action, _ = model.predict(obs)
                    obs, step_reward, episode_done, _ = env.step(pred_action)
                    total_ep_reward += step_reward
                    hit_count += int(obs[6])
                all_episode_total_rewards += total_ep_reward
                all_episode_hit_ratio += (hit_count/step_count)

            coord_mean_rewards.append(all_episode_total_rewards/NUM_EPISODES)
            coord_mean_hit_probs.append(all_episode_hit_ratio/NUM_EPISODES)
        
        np.savetxt(f'{save_path}/mean_reward_{MODEL_NAME}_{MODEL_TO_LOAD}_{NUM_EPISODES}eps.txt', np.array(coord_mean_rewards))
        np.savetxt(f'{save_path}/mean_hit_prob_{MODEL_NAME}_{MODEL_TO_LOAD}_{NUM_EPISODES}eps.txt', np.array(coord_mean_hit_probs))
        rospy.loginfo(f"Evaluation result saved to {save_path}!")

    except Exception as e:
        env.close()
        rospy.logerr(f"Exiting due to reason: {e}")
        rospy.signal_shutdown("Exception occurred")

    env.close()


def set_model_state(x, y):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        robot_pose = ModelState()
        robot_pose.model_name = 'jackal0'
        robot_pose.reference_frame = "/map"
        robot_pose.pose.position.x = x
        robot_pose.pose.position.y = y
        robot_pose.pose.position.z = 0
        robot_pose.pose.orientation.x = 0
        robot_pose.pose.orientation.y = 0
        robot_pose.pose.orientation.z = 0
        robot_pose.pose.orientation.w = 1
        reset_gazebo_simulation_proxy(robot_pose)
        return True
    except rospy.ServiceException as e:
        print ("/gazebo/set_model_state service call failed")
        return False


if __name__ == '__main__':
    main()
