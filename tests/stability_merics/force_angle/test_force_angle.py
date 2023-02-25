from typing import List, Tuple

import pytest
from seer.stability_metrics.force_angle.config import ForceAngleConfig
from seer.stability_metrics.force_angle.force_angle import ForceAngle
from seer.stability_metrics.force_angle.state import ForceAngleState


@pytest.mark.parametrize(
    "config,states_and_expected_force_angles",
    [
        (
            ForceAngleConfig(
                clockwise_ground_contact_points=[
                    [-1, -1, 0],
                    [-1, 1, 0],
                    [1, 1, 0],
                    [1, -1, 0]
                ]
            ),
            [
                (ForceAngleState(centre_of_mass=[0,0,0], reaction_forces=[[0,0,0]], reaction_moments=[[0,0,0]]), 0)
            ]
        )
    ]
)
def test_force_angle(config: ForceAngleConfig,
                     states_and_expected_force_angles: List[Tuple[ForceAngleState, float]]):
    force_angle = ForceAngle.instance(config)
    for state, expected_force_angle in states_and_expected_force_angles:
        assert force_angle.get(state) == expected_force_angle, f"Incorrect force angle for state: {state}"
