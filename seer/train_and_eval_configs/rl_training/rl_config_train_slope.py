from seer.train_and_eval_configs.create_config import create_config, ConfigMode
from seer.train_and_eval_configs.modifiers.with_slope import with_slope

SLOPE_WEIGHTS_PATH = "Reach_HER_slope"
run_params, env_params, algo_params = create_config(
    mode=ConfigMode.TRAIN,
    scenario_name="slope",
    weights_path=SLOPE_WEIGHTS_PATH,
    modifiers=[with_slope]
)
wandb_config = dict()
wandb_config.update(run_params)
wandb_config.update(env_params)
wandb_config.update(algo_params)
