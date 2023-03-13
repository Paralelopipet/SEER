from seer.train_and_eval_configs.create_config import Controller, create_config, ConfigMode
from seer.train_and_eval_configs.modifiers.with_noise import with_noise
from seer.train_and_eval_configs.rl_training.rl_config_train_basic import BASIC_WEIGHTS_PATH

run_params, env_params, algo_params = create_config(
    mode=ConfigMode.TRAIN,
    scenario_name="eval noisy - controller basic",
    weights_path=BASIC_WEIGHTS_PATH,
    modifiers=[with_noise],
    controller=Controller.RL
)
wandb_config = dict()
wandb_config.update(run_params)
wandb_config.update(env_params)
wandb_config.update(algo_params)
