from seer.train_and_eval_configs.rl_eval.defaults import default_run_params
from seer.train_and_eval_configs.rl_eval.rl_config_eval_noisy import env_params as noisy_env_params
from seer.train_and_eval_configs.rl_eval.rl_config_eval_noisy import algo_params as noisy_algo_params
from seer.train_and_eval_configs.constants import *
from seer.train_and_eval_configs.rl_training.rl_config_train_noisy_spring import run_params as noisy_spring_train_run_params
wandb_config = dict()

run_params = default_run_params.copy()
run_params.update( {
    SCENARIO: 'eval noisy - controller noisy spring',
    PATH: noisy_spring_train_run_params[PATH]
})
wandb_config.update(run_params)


env_params = noisy_env_params.copy()
wandb_config.update(env_params)


algo_params = noisy_algo_params.copy()
wandb_config.update(algo_params)

