from enum import Enum
from typing import TYPE_CHECKING, Type

if TYPE_CHECKING:
    from seer.stability_metrics.adapter.config_adapter import ConfigAdapter
    from seer.stability_metrics.adapter.state_adapter import StateAdapter
    from seer.stability_metrics.stability_metric import StabilityMetric


class StabilityMetrics(Enum):
    force_angle = "force_angle"

    @property
    def implementation(self) -> Type["StabilityMetric"]:
        from seer.stability_metrics.force_angle.force_angle import ForceAngle
        if self == StabilityMetrics.force_angle:
            return ForceAngle
        else:
            raise NotImplementedError()
    
    @property
    def config_adapter(self) -> Type["ConfigAdapter"]:
        from seer.stability_metrics.adapter.force_angle.config_adapter import ForceAngleConfigAdapter
        if self == StabilityMetrics.force_angle:
            return ForceAngleConfigAdapter
        else:
            raise NotImplementedError()
        
    @property
    def state_adapter(self) -> Type["StateAdapter"]:
        from seer.stability_metrics.adapter.force_angle.state_adapter import ForceAngleStateAdapter
        if self == StabilityMetrics.force_angle:
            return ForceAngleStateAdapter
        else:
            raise NotImplementedError()
    
