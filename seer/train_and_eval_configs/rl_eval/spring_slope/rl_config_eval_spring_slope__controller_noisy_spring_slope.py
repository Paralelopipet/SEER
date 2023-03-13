from seer.train_and_eval_configs.create_config import Controller, create_config, ConfigMode
from seer.train_and_eval_configs.modifiers.with_slope import with_slope
from seer.train_and_eval_configs.modifiers.with_spring import with_spring
from seer.train_and_eval_configs.rl_training.rl_config_train_noisy_spring_slope import NOISY_SPRING_SLOPE_WEIGHTS_PATH

run_params, env_params, algo_params = create_config(
    mode=ConfigMode.EVAL,
    scenario_name="eval spring slope - controller noisy spring slope",
    weights_path=NOISY_SPRING_SLOPE_WEIGHTS_PATH,
    modifiers=[with_spring, with_slope],
    controller=Controller.RL
)
wandb_config = dict()
wandb_config.update(run_params)
wandb_config.update(env_params)
wandb_config.update(algo_params)
