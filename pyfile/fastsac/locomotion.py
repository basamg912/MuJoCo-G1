"""Basic locomotion observation terms.

These functions compute individual observation components for legged locomotion tasks.
Each function mirrors the manager-based observation pipeline that replaced the legacy direct `_get_obs_*()` helpers.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, cast

import torch

from holosoma.utils.rotations import quat_rotate_inverse
from holosoma.utils.torch_utils import get_axis_params, to_torch

if TYPE_CHECKING:
    from holosoma.envs.locomotion.locomotion_manager import LeggedRobotLocomotionManager
    from holosoma.managers.command.terms.locomotion import LocomotionGait


# ! isaaclab 기준 observation 정리
# * self.base_quat = self.robot_root_states[:, 3:7]  # (num_envs, 4), xyzw
def _base_quat(env: LeggedRobotLocomotionManager) -> torch.Tensor:
    return env.base_quat


# * 0 0 -1 상수값 으로 사용
def gravity_vector(env: LeggedRobotLocomotionManager, up_axis_idx: int = 2) -> torch.Tensor:
    axis = to_torch(get_axis_params(-1.0, up_axis_idx), device=env.device)
    return axis.unsqueeze(0).expand(env.num_envs, -1)

def base_forward_vector(env: LeggedRobotLocomotionManager) -> torch.Tensor:
    axis = to_torch([1.0, 0.0, 0.0], device=env.device)
    return axis.unsqueeze(0).expand(env.num_envs, -1)


'''
# * self.robot_root_states = RootStatesProxy(self._robot.data.root_state_w)  # (num_envs, 13)
RootStateProxy class -> reset
    def reset(self, tensor_wxyz: torch.Tensor):
        self.tensor_wxyz = tensor_wxyz
        self.tensor_xyzw = fullstate_wxyz_to_xyzw(tensor_wxyz)
! xyzw -> wxyz convert : pytorch 등 라이브러리에서 사용
'''
def get_base_lin_vel(env: LeggedRobotLocomotionManager) -> torch.Tensor:
    root_states = env.simulator.robot_root_states
    lin_vel_world = root_states[:, 7:10]
    return quat_rotate_inverse(_base_quat(env), lin_vel_world, w_last=True)


def get_base_ang_vel(env: LeggedRobotLocomotionManager) -> torch.Tensor:
    ang_vel_world = env.simulator.robot_root_states[:, 10:13]
    return quat_rotate_inverse(_base_quat(env), ang_vel_world, w_last=True)


def get_projected_gravity(env: LeggedRobotLocomotionManager) -> torch.Tensor:
    return quat_rotate_inverse(_base_quat(env), gravity_vector(env), w_last=True)


def base_lin_vel(env: LeggedRobotLocomotionManager) -> torch.Tensor:
    """Base linear velocity in base frame.

    Returns:
        Tensor of shape [num_envs, 3]

    Equivalent to:
        env._get_obs_base_lin_vel()
    """
    return get_base_lin_vel(env)


def base_ang_vel(env: LeggedRobotLocomotionManager) -> torch.Tensor:
    """Base angular velocity in base frame.
