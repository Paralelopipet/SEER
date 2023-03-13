import argparse
import importlib
from seer.train_and_eval_configs.constants import CLASSIC, CONTROLLER, RL
from pybullet_multigoal_implementation.drl_implementation.examples.FetchReachHER import main as rl_main
from seer.train_and_eval_configs.baseline.wandb_local_baseline import wandb_run as baseline_main


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--use-wandb', default=False, action='store_true',
                        help='Flag to enable or disable wandb. Default: False.')
    parser.add_argument("--config",
                        required=True,
                        help="The complete path to the config file to use, for example seer.train_and_eval_configs.rl_eval.rl_config_eval_basic")
    parser = parser.parse_args()
    
    print("Using wandb?", parser.use_wandb)
    config = importlib.import_module(parser.config)
    if config.run_params[CONTROLLER] == CLASSIC:
        baseline_main(parser.use_wandb, config)
    elif config.run_params[CONTROLLER] == RL:
        rl_main(parser.use_wandb, config)
    else:
        raise NotImplementedError()
