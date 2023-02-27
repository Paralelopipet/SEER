
from ..adapter.config_adapter import ConfigAdapter
from ..adapter.types import RobotConfig
from .config import ForceAngleConfig


class ForceAngleConfigAdapter(ConfigAdapter[ForceAngleConfig]):
    @classmethod
    def convert(cls, to_convert: RobotConfig) -> ForceAngleConfig:
        # TODO
        return to_convert
