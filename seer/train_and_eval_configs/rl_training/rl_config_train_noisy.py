from seer.train_and_eval_configs.constants import *
from seer.train_and_eval_configs.rl_training.defaults import default_run_params, default_env_params, default_algo_params
wandb_config = dict()


run_params = default_run_params.copy()
run_params.update( {
    SCENARIO: 'noisy action and maybe observation',
    PATH: 'Reach_HER_noisy'
})
wandb_config.update(run_params)


env_params = default_env_params.copy()
env_params.update(
    # plane_position = [0.,0.,-0.40], # with spring
    plane_position = [0., 0., -0.58],  # without spring
    # has_spring = True,
    tip_penalty = -30.0,  # -20.0
    tipping_threshold=0.5, 
    noise_stds = {
        'pos' : 100.0, # 0.0
        'vel' : 100.0, # 0.0
        'tor' : 100.0, # 0.0
        'com' : 100.0, # 0.0
    }
)
wandb_config.update(env_params)


algo_params = default_algo_params.copy()
algo_params.update({
    'action_noise_std': 1.0, # 0.0
    'observation_noise_std' : env_params['noise_stds']
})
wandb_config.update(algo_params)

