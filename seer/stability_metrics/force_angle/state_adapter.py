from ..adapter.types import RobotState
from ..adapter.state_adapter import StateAdapter
from .force_angle import ForceAngleState

class ForceAngleStateAdapter(StateAdapter[ForceAngleState]):
    @classmethod
    def convert(cls, to_convert: RobotState) -> ForceAngleState:
        # TODO
        return to_convert
