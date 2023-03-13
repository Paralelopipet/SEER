from enum import Enum
from typing import Callable, List, Union

from seer.train_and_eval_configs.constants import *
from seer.train_and_eval_configs.defaults import (default_rl_algo_params,
                                                  default_env_params,
                                                  default_run_params)


class ConfigMode(Enum):
    EVAL = "EVAL"
    TRAIN = "TRAIN"

class Controller(Enum):
    CLASSIC = "classic"
    RL = "rl"

def create_config(*,
    mode: ConfigMode,
    scenario_name: str,
    controller: Controller,
    weights_path: Union[str, None] = None,
    modifiers: List[Callable] = []
 ):
    if mode == ConfigMode.EVAL:
        run_params, env_params = _eval_base_config()
    elif mode == ConfigMode.TRAIN:
        run_params, env_params = _train_base_config()
    else:
        raise NotImplementedError()

    if controller == Controller.CLASSIC:
        algo_params = _classic_base_config(run_params)
    elif controller == Controller.RL:
        algo_params = _rl_base_config(run_params)
    else:
        raise NotImplementedError()
    
    run_params[SCENARIO] = scenario_name
    run_params[PATH] = weights_path

    for modifier in modifiers:
        modifier(run_params=run_params,
                 env_params=env_params,
                 algo_params=algo_params)
    return run_params, env_params, algo_params


def _eval_base_config():
    run_params = default_run_params.copy()
    env_params = default_env_params.copy()
    run_params[IS_TEST] = True
    run_params[LOAD_NETWORK_EP] = 10
    env_params["target_min_distance_xy"] = 0.23
    env_params["checkReachability"] = True
    return run_params, env_params

def _train_base_config():
    run_params = default_run_params.copy()
    env_params = default_env_params.copy()
    run_params[IS_TEST] = False
    run_params[LOAD_NETWORK_EP] = None
    env_params["target_min_distance_xy"] = 0.1
    return run_params, env_params

def _rl_base_config(run_params):
    run_params.update({
        CONTROLLER: RL,
        ARCHITECTURE: 'ddpg-goal-conditioned',
        TRAINING_SCRIPT: 'FetchReachHER',
    })
    return default_rl_algo_params.copy()

def _classic_base_config(run_params):
    run_params.update({
        CONTROLLER: CLASSIC,
        ARCHITECTURE: 'force-angle',
        TRAINING_SCRIPT: 'baseline_experiment',
    })
    return {}
