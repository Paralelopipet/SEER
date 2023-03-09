import wandb

# Switch with your own wandb_config containing everything you want to track
from rl_configs import wandb_config as rl_wandb_config
# Import the script you want to run
from pybullet_multigoal_implementation.drl_implementation.examples.FetchReachHER import main as train

# start a new wandb run to track this script
wandb.init(
    # set the wandb project where this run will be logged
    project='seer',
    entity='general-team',
    
    # track hyperparameters and run metadata
    config=rl_wandb_config
)

# Run the desired script
train()
