from abc import ABC, abstractclassmethod, abstractmethod
from typing import Generic, TypeVar

Config = TypeVar("Config")
State = TypeVar("State")

class StabilityMetric(ABC, Generic[Config, State]):
    @abstractclassmethod
    def instance(cls, config: Config) -> "StabilityMetric":
        pass

    @abstractmethod
    def get(self, state: State) -> float:
        pass
