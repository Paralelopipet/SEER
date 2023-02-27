from typing import Generic, Type, TypeVar


from ..stability_metric import StabilityMetric
from ..force_angle import ForceAngle, ForceAngleConfigAdapter, ForceAngleStateAdapter
from .config_adapter import ConfigAdapter
from .state_adapter import StateAdapter
from .types import RobotConfig, RobotState


ImplConfig = TypeVar("ImplConfig")
ImplState = TypeVar("ImplState")

class StabilityMetricAdapter(StabilityMetric[RobotConfig, RobotState], Generic[ImplConfig, ImplState]):
    DEFAULT_IMPLEMENTATION = ForceAngle
    DEFAULT_CONFIG_ADAPTER = ForceAngleConfigAdapter
    DEFAULT_STATE_ADAPTER = ForceAngleStateAdapter

    def __init__(self,
                 config: RobotConfig,
                 implementation: Type[StabilityMetric[ImplConfig, ImplState]],
                 config_adapter: Type[ConfigAdapter[ImplConfig]],
                 state_adapter: Type[StateAdapter[ImplState]]):
        self._implementation = implementation.instance(config_adapter.convert(config))
        self._state_adapter = state_adapter

    def get(self, state: RobotState) -> float:
        return self._implementation.get(self._state_adapter.convert(state))

    @classmethod
    def instance(cls,
                 config: RobotConfig) -> StabilityMetric:
        return cls(config,
                   cls.DEFAULT_IMPLEMENTATION,
                   cls.DEFAULT_CONFIG_ADAPTER,
                   cls.DEFAULT_STATE_ADAPTER)
