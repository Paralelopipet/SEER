wandb_config = dict()

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
SCENARIO = 'scenario'
SPRING_FORCE = 'spring_force'
BOX_MASS = 'box_mass'

run_params = {
    CONTROLLER: CLASSIC,  # rl or classic
    ARCHITECTURE: 'force-angle', # specify style of controller
    TRAINING_SCRIPT: 'baseline_experiment',
    SCENARIO: 'spring',
    SLEEP: 0.0,
    SEEDS: [11, 22, 33, 44], # [0]
    IS_TEST: False,
    PATH: 'None',
    LOAD_NETWORK_EP: None,
    SPRING_FORCE: 30,
    # BOX_MASS: 10, # Symbolic param, since it is used in .urdf file
}
wandb_config.update(run_params)


env_params = {}
env_params.update(
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
    plane_position = [0.,0.,-0.40], # with spring
    # plane_position = [0., 0., -0.58],  # without spring
    has_spring = True,
    joint_force_sensors=True,
    tip_penalty = -30.0,  # -20.0
    force_angle_reward_factor = 1.0,
    noise_stds = {
        'pos' : 100.0, # 0.0
        'vel' : 100.0, # 0.0
        'tor' : 100.0, # 0.0
        'com' : 100.0, # 0.0
    },
)
wandb_config.update(env_params)


algo_params = {
}
wandb_config.update(algo_params)

