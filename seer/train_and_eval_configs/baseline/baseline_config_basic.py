from seer.train_and_eval_configs.create_config import Controller, create_config, ConfigMode

run_params, env_params, algo_params = create_config(
    mode=ConfigMode.EVAL,
    scenario_name="eval basic - controller baseline",
    controller=Controller.CLASSIC
)
wandb_config = dict()
wandb_config.update(run_params)
wandb_config.update(env_params)
wandb_config.update(algo_params)
