from ..adapter.types import RobotState
from ..adapter.state_adapter import StateAdapter
from .force_angle import ForceAngleState

class ForceAngleStateAdapter(StateAdapter[ForceAngleState]):
    @classmethod
    def convert(cls, to_convert: RobotState) -> ForceAngleState:
        # TODO
        # note that all values in the robot state need to be relative to the cube origin!
        # i.e., rotate and translate the forces and moments and centre of mass by the current orientation/position of the cube origin
        return to_convert
