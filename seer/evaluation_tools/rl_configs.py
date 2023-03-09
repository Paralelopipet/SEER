
wandb_config = dict()

algo_params = {
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
    'action_noise_std': 0.00,

    'training_epochs': 11,
    'training_cycles': 50,
    'training_episodes': 16,
    'testing_gap': 10,
    'testing_episodes': 30,
    'saving_gap': 1,

    # 'cuda_device_id': 0 disable cuda usage
    # 'cuda_device_full_name': 'mps'
}
wandb_config.update(algo_params)

env_params = {}
env_params.update(
    task = 'tip_over',
    gripper = 'parallel_jaw',
    render = True,
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
    # plane_position = [0.,0.,-0.38], # with spring
    plane_position = [0., 0., -0.58],  # without spring
    # has_spring = True,
    tip_penalty = -50.0,  # -20.0
    force_angle_reward_factor = 1.0,
)
wandb_config.update(env_params)

CONTROLLER = 'controller'
RL = 'rl'
CLASSIC = 'classic'
ARCHITECTURE = 'architecture'
TRAINING_SCRIPT = 'training_script'
SLEEP = 'sleep'
SEEDS = 'seeds'
IS_TEST = 'is_test'
PATH = 'path'
LOAD_NETWORK_EP = 'load_network_ep'

run_params = {
    CONTROLLER: RL,  # rl or classic
    ARCHITECTURE: 'ddpg-goal-conditioned',
    TRAINING_SCRIPT: 'FetchReachHER',
    SLEEP: 0.0,
    # SEEDS: [11, 22, 33, 44],
    # SEEDS: [33, 44],
    SEEDS: [0],
    IS_TEST: False,
    PATH: 'Reach_HER',
    LOAD_NETWORK_EP: None,
}
wandb_config.update(run_params)
