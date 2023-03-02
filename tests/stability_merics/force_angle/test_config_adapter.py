from pathlib import Path
from typing import Union

import numpy as np
import pytest

from pybullet_multigoal_gym.pybullet_multigoal_gym.utils.cube_path import \
    CUBE_PATH
from seer.stability_metrics.adapter.types import RobotConfig
from seer.stability_metrics.force_angle.config import ForceAngleConfig
from seer.stability_metrics.force_angle.config_adapter import \
    ForceAngleConfigAdapter

RESOURCE_DIR = Path(__file__).parent / "cubes"

@pytest.mark.parametrize(
    "urdf_path,cube_link_name,expected_config",
    [
        (
            CUBE_PATH,
            "cube",
            ForceAngleConfig(
                clockwise_ground_contact_points=[
                    np.array([0.015, -0.015, -0.015]),
                    np.array([0.015, -0.015, 0.015]),
                    np.array([-0.015, -0.015, 0.015]),
                    np.array([-0.015, -0.015, -0.015])
                ]
            ),
        ),
        (
            RESOURCE_DIR /"other_box.urdf",
            "cube",
            ForceAngleConfig(
                clockwise_ground_contact_points=[
                    np.array([0.03, -0.02, -0.01]),
                    np.array([0.03, -0.02, 0.01]),
                    np.array([-0.03, -0.02, 0.01]),
                    np.array([-0.03, -0.02, -0.01])
                ]
            ),
        ),
    ]
)
def test_force_angle_config_adapter(urdf_path: Union[str, Path], cube_link_name: str, expected_config: ForceAngleConfig):
    assert ForceAngleConfigAdapter.convert(RobotConfig(urdf_path=str(urdf_path), cube_link_name=cube_link_name)) == expected_config


@pytest.mark.parametrize("urdf_path,cube_link_name", [
    (RESOURCE_DIR / "cube_no_collision.urdf", "cube"),
    (RESOURCE_DIR / "cube_with_position.urdf", "cube"),
    (RESOURCE_DIR / "cube_with_rotation.urdf", "cube")
])
def test_invalid_urdfs_raise_error(urdf_path, cube_link_name):
    with pytest.raises(AssertionError):
        ForceAngleConfigAdapter.convert(RobotConfig(urdf_path=str(urdf_path), cube_link_name=cube_link_name))
