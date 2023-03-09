import os
# from drl_implementation.examples import plot
import pybullet_multigoal_gym as pmg
from  pybullet_multigoal_gym.envs.task_envs.kuka_single_step_envs import \
    KukaTipOverEnv
from pybullet_multigoal_implementation.drl_implementation.agent.utils import plot
# import gym
from seer.evaluation_tools.rl_configs import *
from pybullet_multigoal_gym.examples.kuka_controller import run

def main():
    seeds = run_params[SEEDS] #
    seed_returns = []
    seed_success_rates = []
    path = os.path.dirname(os.path.realpath(__file__))
    # path = os.path.join(path, run_params[PATH])

    for seed in seeds:

        env: KukaTipOverEnv = pmg.make_env(**env_params)
        # seed_path = path + '/seed'+str(seed)

        # agent = GoalConditionedDDPG(algo_params=algo_params, env=env, path=seed_path, seed=seed)
        # agent.run(test=run_params[IS_TEST], render=env_params['render'],
        #           load_network_ep=run_params[LOAD_NETWORK_EP], sleep=run_params[SLEEP])
        run(env, seed)
        # agent.run(test=True, load_network_ep=10, sleep=0.05)
        # BUG TODO Verify: if you are loading to continue training, the data (not weights) are only from the last full epoch you saved!
        # agent.run(test=False, load_network_ep=5)
        # seed_returns.append(agent.statistic_dict['epoch_test_return'])
        # seed_success_rates.append(agent.statistic_dict['epoch_test_success_rate'])
        # del env, agent

        # return_statistic = plot.get_mean_and_deviation(seed_returns, save_data=True,
        #                                             file_name=os.path.join(path, 'return_statistic.json'))
        # plot.smoothed_plot_mean_deviation(path + '/returns', return_statistic, x_label='Epoch', y_label='Average returns')


        # success_rate_statistic = plot.get_mean_and_deviation(seed_success_rates, save_data=True,
        #                                                     file_name=os.path.join(path, 'success_rate_statistic.json'))
        # plot.smoothed_plot_mean_deviation(path + '/success_rates', success_rate_statistic,
        #                                 x_label='Epoch', y_label='Success rates')


if __name__ == '__main__':
    main()
