from seer.train_and_eval_configs.create_config import create_config, ConfigMode
from seer.train_and_eval_configs.modifiers.with_noise import with_noise
from seer.train_and_eval_configs.rl_training.rl_config_train_noisy_spring import NOISY_SPRING_WEIGHTS_PATH

run_params, env_params, algo_params = create_config(
    mode=ConfigMode.TRAIN,
    scenario_name="eval noisy - controller noisy spring",
    weights_path=NOISY_SPRING_WEIGHTS_PATH,
    modifiers=[with_noise]
)
wandb_config = dict()
wandb_config.update(run_params)
wandb_config.update(env_params)
wandb_config.update(algo_params)
