from typing import List, Tuple
import numpy as np

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
                    np.array([1, 0, -1]),
                    np.array([1, 0, 1]),
                    np.array([-1, 0, 1]),
                    np.array([-1, 0, -1])
                ]
            ),
            [
                # zero force
                (ForceAngleState(centre_of_mass=np.array([0,1,0]), reaction_forces=[np.array([0,0,0])], reaction_moments=[np.array([0,0,0])]), 0),
                (ForceAngleState(centre_of_mass=np.array([100,-19,20]),
                                 reaction_forces=[np.array([10,15,30]), np.array([-10,-15,-30])],
                                 reaction_moments=[np.array([20,30,40])]), 0),
                # force outside the support polygon
                (ForceAngleState(centre_of_mass=np.array([0,1,0]), reaction_forces=[np.array([0,-1,0])], reaction_moments=[np.array([0,0,0])]), -2.61),
                (ForceAngleState(centre_of_mass=np.array([0,1,0]), reaction_forces=[np.array([0,-1,0])], reaction_moments=[np.array([10,0,0])]), -2.35),
                (ForceAngleState(centre_of_mass=np.array([0,1,0]), reaction_forces=[np.array([0,0,-1])], reaction_moments=[np.array([0,0,0])]), -1.57),
                (ForceAngleState(centre_of_mass=np.array([0,1,0]), reaction_forces=[np.array([-1,0,0])], reaction_moments=[np.array([0,0,0])]), -1.57),
                (ForceAngleState(centre_of_mass=np.array([0,1,0]), reaction_forces=[np.array([-1,0,-1])], reaction_moments=[np.array([0,0,0])]), -1.11),
                (ForceAngleState(centre_of_mass=np.array([0,1,0]), reaction_forces=[np.array([-1,-1,-1])], reaction_moments=[np.array([0,0,0])]), -5.44),
                (ForceAngleState(centre_of_mass=np.array([0,1000,0]), reaction_forces=[np.array([1,10,1])], reaction_moments=[np.array([0,0,0])]), -0.99),
                # force inside the support polygon
                (ForceAngleState(centre_of_mass=np.array([0,1,0]), reaction_forces=[np.array([0,1,0])], reaction_moments=[np.array([0,0,0])]), 0.78),
                (ForceAngleState(centre_of_mass=np.array([0,1,0]), reaction_forces=[np.array([1,2,0])], reaction_moments=[np.array([0,0,0])]), 0.71),
                (ForceAngleState(centre_of_mass=np.array([0,1,0]), reaction_forces=[np.array([0,2,1])], reaction_moments=[np.array([0,0,0])]), 0.71),
                (ForceAngleState(centre_of_mass=np.array([0,1,0]), reaction_forces=[np.array([1,10,1])], reaction_moments=[np.array([0,0,0])]), 6.9),
                # force on the edge of the support polygon
                (ForceAngleState(centre_of_mass=np.array([0,1,0]), reaction_forces=[np.array([1,1,0])], reaction_moments=[np.array([0,0,0])]), 0),
                (ForceAngleState(centre_of_mass=np.array([0,1,0]), reaction_forces=[np.array([0,1,1])], reaction_moments=[np.array([0,0,0])]), 0),
                (ForceAngleState(centre_of_mass=np.array([0,1,0]), reaction_forces=[np.array([1,1,1])], reaction_moments=[np.array([0,0,0])]), 0),
                (ForceAngleState(centre_of_mass=np.array([0,1,0]), reaction_forces=[np.array([-1,1,0])], reaction_moments=[np.array([0,0,0])]), 0)
            ]
        )
    ]
)
def test_force_angle(config: ForceAngleConfig,
                     states_and_expected_force_angles: List[Tuple[ForceAngleState, float]]):
    force_angle = ForceAngle.instance(config)
    for state, expected_force_angle in states_and_expected_force_angles:
        assert force_angle.get(state) == pytest.approx(expected_force_angle, 0.1, 0.01), f"Incorrect force angle for state: {state}"
