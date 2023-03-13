from seer.train_and_eval_configs.create_config import Controller, create_config, ConfigMode
from seer.train_and_eval_configs.modifiers.with_noise import with_noise
from seer.train_and_eval_configs.modifiers.with_slope import with_slope
from seer.train_and_eval_configs.modifiers.with_spring import with_spring

run_params, env_params, algo_params = create_config(
    mode=ConfigMode.EVAL,
    scenario_name="eval noisy spring slope - controller baseline",
    controller=Controller.CLASSIC,
    modifiers=[with_slope, with_spring, with_noise]
)
wandb_config = dict()
wandb_config.update(run_params)
wandb_config.update(env_params)
wandb_config.update(algo_params)
