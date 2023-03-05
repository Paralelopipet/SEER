from typing import Generic, TypeVar


from ..stability_metric import StabilityMetric
from .types import RobotConfig, RobotState


ImplConfig = TypeVar("ImplConfig")
ImplState = TypeVar("ImplState")

class StabilityMetricAdapter(StabilityMetric[RobotConfig, RobotState], Generic[ImplConfig, ImplState]):
    def __init__(self, config: RobotConfig):
        self._implementation = config.metric.implementation.instance(config.metric.config_adapter.convert(config))
        self._state_adapter = config.metric.state_adapter

    def get(self, state: RobotState) -> float:
        return self._implementation.get(self._state_adapter.convert(state))

    @classmethod
    def instance(cls, config: RobotConfig) -> StabilityMetric:
        return cls(config)
