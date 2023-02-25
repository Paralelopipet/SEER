from functools import cached_property
from typing import List

import numpy as np

from seer.utility import normalize_all
from seer.utility.types import Point3D, Vector3D

from ..stability_metric import StabilityMetric
from .config import ForceAngleConfig
from .state import ForceAngleState


class ForceAngle(StabilityMetric[ForceAngleConfig, ForceAngleState]):
    def __init__(self, config: ForceAngleConfig):
        self._config = config
    
    @classmethod
    def instance(cls, config: ForceAngleConfig) -> StabilityMetric:
        return cls(config)

    def get(self, state: ForceAngleState) -> float:
        net_force = self._net_force_or_moment(state.reaction_forces)
        net_moment = self._net_force_or_moment(state.reaction_moments)
        tip_over_angles = self._tip_over_angles(net_force,
                                                net_moment,
                                                state.centre_of_mass)
        return self._force_angle(net_force, tip_over_angles)

    def _force_angle(self, net_force: Vector3D, tip_over_angles: List[float]) -> float:
        return min(tip_over_angles[i] for i in range(self._n)) * np.linalg.norm(net_force)

    def _net_force_or_moment(self, reaction_force_or_moments: List[Vector3D]) -> Vector3D:
        return -sum(reaction_force_or_moments)

    def _tip_over_angles(self,
                         net_force: Vector3D,
                         net_moment: Vector3D,
                         centre_of_mass: Point3D
                         ) -> List[float]:
        tip_over_axis_normals = self._tip_over_axis_normals(centre_of_mass)
        normalized_tip_over_axis_normals = normalize_all(tip_over_axis_normals)
        net_force_about_tip_over_axes = self._net_force_about_tip_over_axes(
            net_force,
            net_moment,
            tip_over_axis_normals,
            normalized_tip_over_axis_normals
        )
        normalized_net_force_about_tip_over_axes = normalize_all(net_force_about_tip_over_axes)
        directions = self._tip_over_angle_directions(normalized_tip_over_axis_normals,
                                                     normalized_net_force_about_tip_over_axes)
        return [
            directions[i] * np.arccos(normalized_net_force_about_tip_over_axes[i].dot(normalized_tip_over_axis_normals[i]))
            for i in range(self._n)
        ]

    def _tip_over_axis_normals(self, centre_of_mass: Point3D):
        return [
            self._to_tip_over_axis_normals[i]
            @ (self._config.clockwise_ground_contact_points[(i + 1) % self._n] - centre_of_mass)
            for i in range(self._n)
        ]

    @cached_property
    def _to_tip_over_axis_normals(self):
        return [
            (np.identity(3) - axis @ axis.T)
            for axis in self._normalized_tip_over_axes
        ]
    
    @cached_property
    def _normalized_tip_over_axes(self):
        return normalize_all(self._tip_over_axes)

    @cached_property
    def _tip_over_axes(self) -> List[Vector3D]:
        return [
            self._config.clockwise_ground_contact_points[(i + 1) % self._n]
            - self._config.clockwise_ground_contact_points[i]
            for i in range(self._n)
        ]

    def _tip_over_angle_directions(self,
                                   normalized_tip_over_axis_normals: List[Vector3D],
                                   normalized_net_force_about_tip_over_axes: List[Vector3D]) -> int:
        return [
            1
            if np.cross(
                   normalized_tip_over_axis_normals[i],
                   normalized_net_force_about_tip_over_axes[i]
               ).dot(self._normalized_tip_over_axes[i]) < 0
            else -1
            for i in range(self._n)
        ]

    def _net_force_about_tip_over_axes(self,
                                       net_force: Vector3D,
                                       net_moment: Vector3D,
                                       tip_over_axis_normals: List[Vector3D],
                                       normalized_tip_over_axis_normals: List[Vector3D]) -> Vector3D:
        net_force_about_tip_over_axes = self._net_force_or_moment_about_tip_over_axes(net_force)
        net_moment_about_tip_over_axes = self._net_force_or_moment_about_tip_over_axes(net_moment)
        return [
            net_force_about_tip_over_axes[i]
            + np.cross(normalized_tip_over_axis_normals[i],
                       net_moment_about_tip_over_axes[i])
            / np.linalg.norm(tip_over_axis_normals[i])
            for i in range(self._n)
        ]

    def _net_force_or_moment_about_tip_over_axes(self,
                                                 net_force_or_moment: Vector3D):
        return [
            self._to_tip_over_axis_normals[i] @ net_force_or_moment
            for i in range(self._n)
        ]

    @property
    def _n(self) -> int:
        return len(self._config.clockwise_ground_contact_points)
