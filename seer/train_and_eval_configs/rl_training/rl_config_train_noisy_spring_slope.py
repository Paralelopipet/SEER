from seer.train_and_eval_configs.create_config import Controller, create_config, ConfigMode
from seer.train_and_eval_configs.modifiers.with_noise import with_noise
from seer.train_and_eval_configs.modifiers.with_slope import with_slope
from seer.train_and_eval_configs.modifiers.with_spring import with_spring

NOISY_SPRING_SLOPE_WEIGHTS_PATH = "Reach_HER_noisy_spring_slope"
run_params, env_params, algo_params = create_config(
    mode=ConfigMode.TRAIN,
    scenario_name="noisy spring slope",
    weights_path=NOISY_SPRING_SLOPE_WEIGHTS_PATH,
    modifiers=[with_slope, with_spring, with_noise],
    controller=Controller.RL
)
wandb_config = dict()
wandb_config.update(run_params)
wandb_config.update(env_params)
wandb_config.update(algo_params)
