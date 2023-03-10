import os
# from drl_implementation.examples import plot
import pybullet_multigoal_gym as pmg
from  pybullet_multigoal_gym.envs.task_envs.kuka_single_step_envs import \
    KukaTipOverEnv
from pybullet_multigoal_implementation.drl_implementation.agent.utils import plot
# import gym
from seer.evaluation_tools.baseline.baseline_config_spring import *
from pybullet_multigoal_gym.examples.kuka_controller import run

def main():
    seeds = run_params[SEEDS] #
    seed_returns = []
    seed_success_rates = []
    path = os.path.dirname(os.path.realpath(__file__))
    # path = os.path.join(path, run_params[PATH])

    for seed in seeds:

        env: KukaTipOverEnv = pmg.make_env(**env_params)
        run(env, seed)


if __name__ == '__main__':
    main()
