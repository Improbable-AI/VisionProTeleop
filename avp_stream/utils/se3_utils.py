
import torch
import numpy as np
from pytorch3d.transforms import * 
from typing import * 


def isaac_mat2quat(mat: torch.Tensor) -> torch.Tensor:
    """
    returns posquat in pos + xyzw format (following isaacgym convention)
    """
    wxyz = matrix_to_quaternion(mat)
    xyzw = torch.cat([wxyz[:, 1:], wxyz[:, :1]], dim=-1)
    return xyzw

def mat2posquat(mat: torch.Tensor) -> torch.Tensor:
    pos = mat[..., :3, 3]
    quat = isaac_mat2quat(mat[..., :3, :3])
    return torch.cat([pos, quat], dim=-1)

def posquat2mat(posquat: torch.Tensor) -> torch.Tensor:
    """
    convert pos + quaternion in xyzw format to matrix 
    """
    batch = posquat.shape[0]
    pos = posquat[..., :3]
    quat_xyzw = posquat[..., 3:]
    quat_wxyz = torch.cat([quat_xyzw[:, 3:], quat_xyzw[:, :3]], dim=-1)
    rot_mat = quaternion_to_matrix(quat_wxyz)

    result = torch.eye(4, device=pos.device, dtype=pos.dtype).unsqueeze(0).expand(batch, -1, -1)
    result[..., :3, :3] = rot_mat
    result[..., :3, 3] = pos

    return result

