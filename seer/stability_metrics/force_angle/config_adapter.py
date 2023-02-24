
from ..adapter import config_adapter
from ..adapter.types import RobotConfig
from .config import ForceAngleConfig


class ForceAngleConfigAdapter(config_adapter[ForceAngleConfig]):
    @classmethod
    def convert(cls, to_convert: RobotConfig) -> ForceAngleConfig:
        # TODO
        return to_convert
