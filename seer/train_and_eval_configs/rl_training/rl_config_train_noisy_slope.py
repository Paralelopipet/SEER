from seer.train_and_eval_configs.create_config import create_config, ConfigMode
from seer.train_and_eval_configs.modifiers.with_slope import with_slope
from seer.train_and_eval_configs.modifiers.with_noise import with_noise

NOISY_SLOPE_WEIGHTS_PATH = "Reach_HER_noisy_slope"
run_params, env_params, algo_params = create_config(
    mode=ConfigMode.TRAIN,
    scenario_name="noisy slope",
    weights_path=NOISY_SLOPE_WEIGHTS_PATH,
    modifiers=[with_slope, with_noise]
)
wandb_config = dict()
wandb_config.update(run_params)
wandb_config.update(env_params)
wandb_config.update(algo_params)