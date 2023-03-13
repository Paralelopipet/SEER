from seer.train_and_eval_configs.create_config import Controller, create_config, ConfigMode

BASIC_WEIGHTS_PATH = "Reach_HER"
run_params, env_params, algo_params = create_config(
    mode=ConfigMode.TRAIN,
    scenario_name="basic",
    weights_path=BASIC_WEIGHTS_PATH,
    controller=Controller.RL
)
wandb_config = dict()
wandb_config.update(run_params)
wandb_config.update(env_params)
wandb_config.update(algo_params)
