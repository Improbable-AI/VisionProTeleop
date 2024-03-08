# subscriber.py

from avp_stream import VisionProStreamer
import argparse 
from typing import * 

if __name__ == "__main__": 

    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', type = str, required = True)
    parser.add_argument('--record', action = 'store_true')
    args = parser.parse_args()

    s = VisionProStreamer(args.ip, args.record)

    while True:
        latest = s.latest
        print(latest['head'][:, :3, -1], latest['right_wrist'][:, :3, -1], latest['right_fingers'].shape)
