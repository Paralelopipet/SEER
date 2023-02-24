from typing import Type, TypeVar

from seer.utility import Adapter

from .types import RobotState


ImplState = TypeVar("ImplState")

StateAdapter = Type[Adapter[RobotState, ImplState]]
