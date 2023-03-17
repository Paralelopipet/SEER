from pathlib import Path
import wandb
import pybullet_multigoal_gym as pmg

# Import the script you want to run
# from pybullet_multigoal_gym.examples.kuka_controller import run
from pybullet_multigoal_gym.examples.kuka_controller import run
from  pybullet_multigoal_gym.envs.task_envs.kuka_single_step_envs import \
    KukaTipOverEnv
# from baseline_experiment import main as run
import argparse
from seer.train_and_eval_configs.constants import DEFAULT_SEEDS, SCENARIO, SEED, SEEDS
# start a new wandb run to track this script

def wandb_run(use_wandb: bool, config):
    seeds = DEFAULT_SEEDS
    env_params = config.env_params
    for seed in seeds:
        if use_wandb:
            wandb.init(
                # set the wandb project where this run will be logged
                project='seer',
                entity='general-team',
                reinit=True,
                name=str(config.run_params[SCENARIO]),
                # track hyperparameters and run metadata
                config=config.wandb_config
            )

        if wandb.run:
            wandb.log({
                SEED: seed,
                SEEDS: seeds,
            })
        env: KukaTipOverEnv = pmg.make_env(**env_params)
        run(env,
            seed=seed,
            num_epochs=config.algo_params['training_epochs'],
            num_cycles=config.algo_params['training_cycles'],
            num_episodes=config.algo_params['training_episodes'],
            action_noise_std=config.algo_params['action_noise_std'])
