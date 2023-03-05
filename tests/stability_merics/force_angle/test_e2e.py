from time import time
from typing import Iterator

import numpy as np
import pytest

import pybullet_multigoal_gym as pmg
from pybullet_multigoal_gym.envs.task_envs.kuka_single_step_envs import \
    KukaTipOverEnv


@pytest.fixture
def pybullet_env() -> Iterator[KukaTipOverEnv]:
    env: KukaTipOverEnv = pmg.make_env(task='tip_over',
                                       gripper='parallel_jaw_cube',
                                       render=False,
                                       binary_reward=True,
                                       joint_control=True,
                                       max_episode_steps=50,
                                       image_observation=False,
                                       depth_image=False,
                                       goal_image=False,
                                       visualize_target=True,
                                       camera_setup=None,
                                       observation_cam_id=[0],
                                       goal_cam_id=0)
    env.reset()
    yield env
    env.close()


def test_detects_tipover(pybullet_env: KukaTipOverEnv):
    EXPECT_TIPOVER_AFTER_S = 5
    tip_over_action = np.ones_like(pybullet_env.action_space.sample())
    fail_after = time() + EXPECT_TIPOVER_AFTER_S
    while time() < fail_after: 
        obs, _, done, _ = pybullet_env.step(tip_over_action)
        if done:
            # TODO this is a hack to make the robot tip over.
            # We should remove it once we have a legitimate tip over movement series
            pybullet_env.reset()
        if obs["tipped_over"]:
            return
    pytest.fail(f"did not detect tipover")


def test_does_not_detect_tipover_in_standstill(pybullet_env):
    EXPECT_NO_TIPOVER_FOR_S = 5
    tip_over_action = np.zeros_like(pybullet_env.action_space.sample())
    pass_after = time() + EXPECT_NO_TIPOVER_FOR_S
    while time() < pass_after: 
        obs, _, _, _ = pybullet_env.step(tip_over_action)
        if obs["tipped_over"]:
            pytest.fail(f"detected tipover")
