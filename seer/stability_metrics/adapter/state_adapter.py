from typing import Type, TypeVar

from seer.utility import Adapter

from .types import RobotState


ImplState = TypeVar("ImplState")

class StateAdapter(Adapter[RobotState, ImplState]):
    pass
