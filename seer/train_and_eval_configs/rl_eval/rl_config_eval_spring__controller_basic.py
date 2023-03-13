from seer.train_and_eval_configs.rl_eval.defaults import default_run_params
from seer.train_and_eval_configs.constants import *
from seer.train_and_eval_configs.rl_eval.rl_config_eval_spring import env_params as spring_env_params
from seer.train_and_eval_configs.rl_eval.rl_config_eval_spring import algo_params as spring_algo_params
from seer.train_and_eval_configs.rl_training.rl_config_train_basic import run_params as basic_train_run_params
wandb_config = dict()


run_params = default_run_params.copy()
run_params.update( {
    SCENARIO: 'eval spring - controller basic',
    PATH: basic_train_run_params[PATH]
})
wandb_config.update(run_params)


env_params = spring_env_params.copy()
wandb_config.update(env_params)


algo_params = spring_algo_params.copy()
wandb_config.update(algo_params)

