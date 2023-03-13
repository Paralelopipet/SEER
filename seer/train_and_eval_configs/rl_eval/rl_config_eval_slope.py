from seer.train_and_eval_configs.create_config import Controller, create_config, ConfigMode
from seer.train_and_eval_configs.modifiers.with_slope import with_slope
from seer.train_and_eval_configs.rl_training.rl_config_train_slope import SLOPE_WEIGHTS_PATH

run_params, env_params, algo_params = create_config(
    mode=ConfigMode.EVAL,
    scenario_name="eval basic - controller slope",
    weights_path=SLOPE_WEIGHTS_PATH,
    controller=Controller.RL,
    modifiers=[with_slope]
)
wandb_config = dict()
wandb_config.update(run_params)
wandb_config.update(env_params)
wandb_config.update(algo_params)
