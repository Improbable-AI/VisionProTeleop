from avp_stream.isaac_env import IsaacVisualizerEnv
from avp_stream import VisionProStreamer
import time 
from typing import * 
import numpy as np 
import torch 

class IsaacVisualizer:

    def __init__(self, args): 
        self.s = VisionProStreamer(args.ip, args.record)
        self.env = IsaacVisualizerEnv(args)

    def run(self):

        while True: 
            latest = self.s.latest
            self.env.step(np2tensor(latest, self.env.device)) 


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
    parser.add_argument('--follow', action = 'store_true', help = "The viewpoint follows the users head")
    args = parser.parse_args()

    vis = IsaacVisualizer(args)
    vis.run()