from seer.train_and_eval_configs.rl_eval.defaults import default_run_params
from seer.train_and_eval_configs.constants import *
from seer.train_and_eval_configs.rl_eval.rl_config_eval_basic import env_params as basic_env_params
from seer.train_and_eval_configs.rl_eval.rl_config_eval_basic import algo_params as basic_algo_params
from seer.train_and_eval_configs.rl_training.rl_config_train_noisy import run_params as noisy_train_run_params
wandb_config = dict()


run_params = default_run_params.copy()
run_params.update( {
    SCENARIO: 'eval basic - controller noisy',
    PATH: noisy_train_run_params[PATH]
})
wandb_config.update(run_params)


env_params = basic_env_params.copy()
wandb_config.update(env_params)


algo_params = basic_algo_params.copy()
wandb_config.update(algo_params)

