#!/usr/bin/env python
import multiAgentChallengeTaskEnv
import gym
from stable_baselines3 import PPO
import os
import rospy
import rospkg

CHECKPOINT_TIMESTEPS = 5000
MODEL_NAME        = 'PPO_stage1'
RESUME_FROM_MODEL = '680000_steps_grav25_move_y1_cl_350k.zip'  # Set to None to create a new model instead of loading
SAVE_MODEL_STRING = 'grav25_move_y1_cl_350k'
TB_LOG_NAME       = 'PPO_stage1_grav25_move_y1_cl_350k'
MODEL_LR          = 0.0003
MODEL_N_STEPS     = 256


def main():
    rospy.init_node('train_particle_shooter_node', anonymous=False, log_level=rospy.DEBUG)

    # Tensorboard logging
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('fyp_rl')
    models_dir = os.path.join(pkg_path, 'training_results', MODEL_NAME)
    tb_log_dir = os.path.join(pkg_path, 'training_results', 'tb_logs')

    if not os.path.exists(models_dir):
        os.makedirs(models_dir)
    if not os.path.exists(tb_log_dir):
        os.makedirs(tb_log_dir)


    env = gym.make('ParticleShooterEnv-v1')
    env.reset()

    model = None

    try:
        if RESUME_FROM_MODEL is not None:
            model_ckpt_path = os.path.join(models_dir, RESUME_FROM_MODEL)
            model = PPO.load(model_ckpt_path, env, verbose=1, tensorboard_log=tb_log_dir,
                            learning_rate=MODEL_LR, n_steps=MODEL_N_STEPS)
            rospy.logwarn(f"Loaded model: {model_ckpt_path}")
        else:
            model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=tb_log_dir,
                        learning_rate=MODEL_LR, n_steps=MODEL_N_STEPS)
            rospy.logwarn(f"Created a new model: {MODEL_NAME}")
    except Exception as e:
        rospy.logerr(f"Exception occurred while loading model: {e}")
        rospy.signal_shutdown("Exception occurred while loading model")
        return


    rospy.logwarn(f"STARTING MODEL TRAINING")
    next_checkpoint = CHECKPOINT_TIMESTEPS if RESUME_FROM_MODEL is None else (int(RESUME_FROM_MODEL.split('_')[0]) + CHECKPOINT_TIMESTEPS)
    try:
        while True:
            model.learn(total_timesteps=CHECKPOINT_TIMESTEPS, reset_num_timesteps=False, tb_log_name=TB_LOG_NAME)
            model.save(f"{models_dir}/{next_checkpoint}_steps_{SAVE_MODEL_STRING}")
            rospy.logwarn(f"CHECKPOINT CREATED - {next_checkpoint} steps")
            next_checkpoint += CHECKPOINT_TIMESTEPS
    
    except KeyboardInterrupt:
        rospy.logwarn("TRAINING STOPPED (KeyboardInterrupt)")
        env.close()


if __name__ == '__main__':
    main()
