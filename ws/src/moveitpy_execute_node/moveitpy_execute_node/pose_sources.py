"""
Pose sources: pluggable providers of target and home poses.

Use ConstantPoseSource for demos or fixed targets; slot in GraspGen or
perception-based sources for evaluation and RL.
"""

import math
from abc import ABC, abstractmethod
from typing import Optional

from geometry_msgs.msg import PoseStamped

from .constants import FRAME_ID
from .pose_utils import pose_stamped_from_xyz_hand_rpy


class PoseSource(ABC):
    """Abstract source of target (e.g. pre-grasp) and home poses."""

    @abstractmethod
    def get_target_pose(self) -> PoseStamped:
        """Return the next target pose (e.g. pre-grasp). Caller may set header.stamp."""
        ...

    @abstractmethod
    def get_home_pose(self) -> PoseStamped:
        """Return the home/ready pose. Caller may set header.stamp."""
        ...


class ConstantPoseSource(PoseSource):
    """Fixed target and home poses (e.g. for demos or testing)."""

    def __init__(
        self,
        *,
        target_pose: Optional[PoseStamped] = None,
        home_pose: Optional[PoseStamped] = None,
    ) -> None:
        if target_pose is None:
            target_pose = _default_pre_grasp_pose()
        if home_pose is None:
            home_pose = _default_home_pose()
        self._target = target_pose
        self._home = home_pose

    def get_target_pose(self) -> PoseStamped:
        out = PoseStamped()
        out.header.frame_id = self._target.header.frame_id
        out.pose = self._target.pose
        return out

    def get_home_pose(self) -> PoseStamped:
        out = PoseStamped()
        out.header.frame_id = self._home.header.frame_id
        out.pose = self._home.pose
        return out


def _default_pre_grasp_pose() -> PoseStamped:
    """Pre-grasp above a small cube; hand opening down (180° about X)."""
    return pose_stamped_from_xyz_hand_rpy(
        0.4, 0.0, 0.15,
        math.pi, 0.0, 0.0,
        frame_id=FRAME_ID,
    )


def _default_home_pose() -> PoseStamped:
    """Panda ready pose (from RV Panda Driver)."""
    return pose_stamped_from_xyz_hand_rpy(
        0.307, 0.0, 0.590,
        math.pi, 0.0, 0.0,
        frame_id=FRAME_ID,
    )


def get_default_home_pose() -> PoseStamped:
    """Ready/home TCP pose in ``panda_link0`` (same as demo grasp sequence)."""
    return _default_home_pose()


def get_default_pose_source() -> PoseSource:
    """Return the default pose source (constant pre-grasp + home). Used by demos."""
    return ConstantPoseSource()
