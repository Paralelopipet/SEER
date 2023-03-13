from seer.train_and_eval_configs.rl_eval.defaults import default_run_params, default_env_params, default_algo_params
from seer.train_and_eval_configs.constants import *
from seer.train_and_eval_configs.rl_training.rl_config_train_spring import run_params as spring_train_run_params
wandb_config = dict()


run_params = default_run_params.copy()
run_params.update( {
    SCENARIO: 'eval spring - controller spring',
    PATH: spring_train_run_params[PATH]
})
wandb_config.update(run_params)


env_params = default_env_params.copy()
env_params.update(
    plane_position = [0.,0.,-0.40], # with spring
    # plane_position = [0., 0., -0.58],  # without spring
    has_spring = True,
    noise_stds = {
        'pos' : 0,#100.05, 
        'vel' : 0,#1000.00,
        'tor' : 0,#10000.00,
        'com' : 0,#10000000.00,
    }
)
wandb_config.update(env_params)


algo_params = default_algo_params.copy()
algo_params.update({
    'action_noise_std': 0, # 1.0,
    'observation_noise_std' : env_params['noise_stds']
})
wandb_config.update(algo_params)

