
import xml.etree.ElementTree as ET
from typing import List, Tuple, TypeVar

import numpy as np

from seer.utility.types import Point3D

from ..config_adapter import ConfigAdapter
from ..types import RobotConfig
from ...force_angle.config import ForceAngleConfig

CubeSize = Tuple[float, float, float]
T = TypeVar("T")

class ForceAngleConfigAdapter(ConfigAdapter[ForceAngleConfig]):
    @classmethod
    def convert(cls, to_convert: RobotConfig) -> ForceAngleConfig:
        return ForceAngleConfig(
            clockwise_ground_contact_points=cls._get_clockwise_ground_contact_points(to_convert)
        )

    @classmethod
    def _get_clockwise_ground_contact_points(cls, to_convert: RobotConfig) -> List[Point3D]:
        cube_xml = cls._get_cube_xml(to_convert)
        cls._assert_no_rotation_and_translation(cube_xml)
        size = cls._parse_size(cube_xml)
        return cls._clockwise_ground_contact_points_from(size)

    @classmethod
    def _get_cube_xml(cls, robot_config: RobotConfig):
        urdf_tree = ET.parse(robot_config.urdf_path)
        cube_xml = urdf_tree.find(f".//link[@name='{robot_config.cube_link_name}']//collision")
        assert cube_xml is not None
        return cube_xml
    
    @classmethod
    def _assert_no_rotation_and_translation(cls, cube_xml: ET.Element):
        box = cube_xml.find(".//origin")
        if box is None:
            return
        assert all(value == 0 for value in (cls._parse_float_array(box, "rpy") + cls._parse_float_array(box, "xyz")))

    @classmethod
    def _parse_size(cls, cube_xml: ET.Element) -> CubeSize:
        box = cube_xml.find(".//box")
        assert box is not None
        return cls._get_size(box)
    
    @classmethod
    def _get_size(cls, box: ET.Element) -> CubeSize:
        xdim, ydim, zdim = cls._parse_float_array(box, "size")
        return xdim, ydim, zdim
    
    @classmethod
    def _parse_float_array(cls, element: ET.Element, attribute: str) -> List[float]:
        values = element.get(attribute)
        assert values is not None
        return [float(val) for val in values.split(" ")]

    @classmethod
    def _clockwise_ground_contact_points_from(cls, size: CubeSize) -> List[Point3D]:
        # "size attribute contains the three side lengths of the box. The origin of the box is in its center." from http://wiki.ros.org/urdf/XML/link
        # we assume that in the future all measurements are transformed into the coordinate system of the cube's origin
        xdim, ydim, zdim = size
        x_min = -xdim/2
        x_max = xdim/2
        y_min = -ydim/2
        y_max = ydim/2
        z = -zdim/2
        return [
            # strange pybullet behaviour uses z for height
            np.array([x_max, z, y_min]),
            np.array([x_max, z, y_max]),
            np.array([x_min, z, y_max]),
            np.array([x_min, z, y_min])
        ]
