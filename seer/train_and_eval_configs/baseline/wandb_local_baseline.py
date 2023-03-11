import wandb
import pybullet_multigoal_gym as pmg

# Switch with your own wandb_config containing everything you want to track
import baseline_config_noisy
import baseline_config_basic
import baseline_config_noisy_spring
import baseline_config_spring
import os 
# Import the script you want to run
# from pybullet_multigoal_gym.examples.kuka_controller import run
from pybullet_multigoal_gym.examples.kuka_controller import run
from  pybullet_multigoal_gym.envs.task_envs.kuka_single_step_envs import \
    KukaTipOverEnv
# from baseline_experiment import main as run
import argparse
from seer.train_and_eval_configs.constants import SEED, SEEDS
# start a new wandb run to track this script

def wandb_run(config, run_params, env_params, name, wandb_enabled=True):
    seeds = [11]

    # Run the desired script
    # seeds = run_params[SEEDS] #

    for seed in seeds:
        if wandb_enabled:
            wandb.init(
            project='seer',
            entity='general-team',
            name =f"{name}_{seed}",   
            config=config
            )

        if wandb.run:
            wandb.log({
                SEED: seed,
                SEEDS: seeds,
            })
        env: KukaTipOverEnv = pmg.make_env(**env_params)
        run(env, seed)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', type=str, default='config_basic.', help='JSON file for configuration')
    args = parser.parse_args()
    config = args.config
    if config == "basic":
        wandb_run(baseline_config_basic.wandb_config, baseline_config_basic.run_params, baseline_config_basic.env_params ,"baseline_basic")
    if config == "noisy":
        wandb_run(baseline_config_noisy.wandb_config, baseline_config_noisy.run_params, baseline_config_noisy.env_params ,"baseline_noisy")
    if config == "spring":
        wandb_run(baseline_config_spring.wandb_config,baseline_config_spring.run_params, baseline_config_spring.env_params , "baseline_spring")
    if config == "noisy_spring":
        wandb_run(baseline_config_noisy_spring.wandb_config, baseline_config_noisy_spring.run_params, baseline_config_noisy_spring.env_params ,"baseline_noisy_spring")
    else:
        raise Exception("option not recognised")