import wandb

# Switch with your own wandb_config containing everything you want to track
from baseline_configs import wandb_config as baseline_wandb_config
# Import the script you want to run
# from pybullet_multigoal_gym.examples.kuka_controller import run
from baseline_experiment import main as run
# start a new wandb run to track this script
wandb.init(
    # set the wandb project where this run will be logged
    project='seer',
    entity='general-team',
    
    # track hyperparameters and run metadata
    config=baseline_wandb_config
)

# Run the desired script
run()
