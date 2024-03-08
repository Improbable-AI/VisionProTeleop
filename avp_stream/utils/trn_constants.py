import torch 
from avp_stream.utils.se3_utils import * 


VISIONOS_TO_ISAAC = torch.tensor([[1, 0, 0, 0], 
                                  [0, 0, -1, 0], 
                                  [0, 1, 0, 0],
                                  [0, 0, 0, 1]], dtype = torch.float32).unsqueeze(0)

FRAME_TRANSFORMATION = torch.tensor([[-1, 0, 0],
                                    [0, 0, 1],
                                    [0, 1, 0]], dtype = torch.float32).unsqueeze(0)
FRAME_QUAT = isaac_mat2quat(FRAME_TRANSFORMATION)

FRAME_44 = torch.tensor([[-1, 0, 0, 0.7],
                         [0, 0,  1, 0.9],
                         [0, 1,  0, 1.6],
                         [0, 0, 0, 1]], dtype = torch.float32).unsqueeze(0)


ROTATE_90DEG_AROUND_X = torch.tensor([[1, 0, 0, 0],
                            [0, 0, -1, 0],
                            [0, 1, 0, 0],
                            [0, 0, 0, 1]], dtype = torch.float32).unsqueeze(0)

ROTATE_90DEG_AROUND_Y = torch.tensor([[0, 0, 1, 0],
                            [0, 1, 0, 0],
                            [-1, 0, 0, 0],
                            [0, 0, 0, 1]], dtype = torch.float32).unsqueeze(0)

ROTATE_NEG_90DEG_AROUND_Y = torch.tensor([[0, 0, -1, 0],
                            [0, 1, 0, 0],
                            [1, 0, 0, 0],
                            [0, 0, 0, 1]], dtype = torch.float32).unsqueeze(0)

ROTATE_180DEG_AROUND_Z = torch.tensor([[-1, 0, 0, 0],
                                       [0, -1, 0, 0],
                                       [0, 0, 1, 0],
                                       [0, 0, 0, 1]], dtype=torch.float32).unsqueeze(0)


VISIONOS_RIGHT_HAND_TO_LEAP = ROTATE_90DEG_AROUND_X @ ROTATE_NEG_90DEG_AROUND_Y

VISIONOS_LEFT_HAND_TO_LEAP = ROTATE_90DEG_AROUND_X @ ROTATE_90DEG_AROUND_Y

VISIONOS_RIGHT_FINGERS_TO_LEAP = ROTATE_90DEG_AROUND_Y
