from typing import Type, TypeVar

from seer.utility import Adapter

from .types import RobotConfig


ImplConfig = TypeVar("ImplConfig")

ConfigAdapter = Type[Adapter[RobotConfig, ImplConfig]]
