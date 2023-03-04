import numpy as np

from seer.utility.types import Vector3D

from ...force_angle.force_angle import ForceAngleState
from ..state_adapter import StateAdapter
from ..types import RobotState


class ForceAngleStateAdapter(StateAdapter[ForceAngleState]):
    @classmethod
    def convert(cls, to_convert: RobotState) -> ForceAngleState:
        return ForceAngleState(centre_of_mass=to_convert.centre_of_mass,
                               reaction_forces=[-to_convert.net_force],
                               reaction_moments=[-to_convert.net_moment])
