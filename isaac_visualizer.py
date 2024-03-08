from avp_stream.isaac_env import IsaacVisualizer
from avp_stream import VisionProStreamer
import time 
from typing import * 
import numpy as np 
import torch 

def np2tensor(data: Dict[str, np.ndarray], device) -> Dict[str, torch.Tensor]:  
    for key in data.keys():
        data[key] = torch.tensor(data[key], dtype = torch.float32, device = device)
    return data


if __name__ == "__main__":
    import argparse 
    import os 

    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', type = str, required = True)
    parser.add_argument('--record', action = 'store_true')
    parser.add_argument('--up_axis', type = str, default = 'Z')
    parser.add_argument('--follow', action = 'store_true', help = "The viewpoint follows the users head")
    args = parser.parse_args()

    s = VisionProStreamer(args.ip, args.record, args.up_axis)

    env = IsaacVisualizer(args)
    while True: 
        t0 = time.time()
        latest = s.latest
        env.step(np2tensor(latest, env.device)) 
        # print("right_wrist_roll: ", latest["right_wrist_roll"])
        # print("right_wrist_roll: ", latest["right_wrist_roll"])
        # print(latest["left_pinch_distance"], latest["right_pinch_distance"])
        # print(time.time() - t0)