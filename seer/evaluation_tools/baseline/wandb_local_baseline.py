import wandb

# Switch with your own wandb_config containing everything you want to track
from baseline_config_noisy import wandb_config as config_noisy
from baseline_config_basic import wandb_config as config_basic
from baseline_config_spring import wandb_config as config_spring
from baseline_config_noisy_spring import wandb_config as config_noisy_spring
# Import the script you want to run
# from pybullet_multigoal_gym.examples.kuka_controller import run
from baseline_experiment import main as run
import argparse
# start a new wandb run to track this script

def wandb_run(config, name):
    wandb.init(
    # set the wandb project where this run will be logged
    project='seer',
    entity='general-team',
    name =name,   
    
    # track hyperparameters and run metadata
    config=config
)

    # Run the desired script
    run()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', type=str, default='config_basic.', help='JSON file for configuration')
    args = parser.parse_args()
    config = args.config
    if config == "basic":
        wandb_run(config_basic, "baseline_basic")
    if config == "noisy":
        wandb_run(config_noisy, "baseline_noisy")
    if config == "spring":
        wandb_run(config_spring, "baseline_spring")
    if config == "noisy_spring":
        wandb_run(config_noisy_spring, "baseline_noisy_spring")
    else:
        raise Exception("option not recognised")