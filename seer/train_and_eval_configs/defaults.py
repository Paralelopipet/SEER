from seer.train_and_eval_configs.constants import *

default_run_params = {
    CONTROLLER: RL,  # rl or classic
    ARCHITECTURE: 'ddpg-goal-conditioned', # specify style of controller
    TRAINING_SCRIPT: 'FetchReachHER',
    SLEEP: 0.0,
    SPRING_FORCE: 30,
    # BOX_MASS: 10, # Symbolic param, since it is used in .urdf file
}

default_env_params = {}
default_env_params.update(
    task = 'tip_over',
    gripper = 'parallel_jaw',
    render = False,
    binary_reward = True,  # Switch to true for stable reward
    joint_control = True,
    max_episode_steps = 50,
    image_observation = False,
    depth_image = False,
    goal_image = False,
    visualize_target = True,
    camera_setup = None,
    observation_cam_id = [0],
    goal_cam_id = 0,
    target_range = 0.5,
    plane_position = [0., 0., -0.58],
    tip_penalty = -30.0,
    tipping_threshold=0.5, 
    noise_stds = {
        'pos' : 0.0,
        'vel' : 0.0,
        'tor' : 0.0,
        'com' : 0.0
    },
    joint_force_sensors=True,
    force_angle_reward_factor = 1.0,
    target_min_distance = 0.3
)

default_algo_params = {
    'hindsight': True,
    'her_sampling_strategy': 'future',
    # choose what type of HER buffer (see PrioritisedHindsightReplayBuffer)
    'prioritised': False,
    'memory_capacity': int(1e6),
    'actor_learning_rate': 0.001,
    'critic_learning_rate': 0.001,
    'Q_weight_decay': 0.0,
    'update_interval': 1,
    'batch_size': 128,
    'optimization_steps': 40,
    'tau': 0.05,
    'discount_factor': 0.98,
    'clip_value': 50,
    'discard_time_limit': True,
    'terminate_on_achieve': True,
    'observation_normalization': True,

    'random_action_chance': 0.2,
    'noise_deviation': 0.05,
    'action_noise_std': 0.0,
    'observation_noise_std': default_env_params['noise_stds'],

    "training_epochs": 11,
    'training_cycles': 50,
    'training_episodes': 16,
    'testing_gap': 10,
    'testing_episodes': 30,
    'saving_gap': 1

    # 'cuda_device_id': 0 disable cuda usage
    # 'cuda_device_full_name': 'mps'
}
