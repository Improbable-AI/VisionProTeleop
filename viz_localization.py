    
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

from avp_stream import VisionProStreamer
from threading import Thread

class LocalizationVisualizer:

    def __init__(self, args): 

        self.args = args 

        self.s = VisionProStreamer(args.ip, args.record)

    def background_localization_visualization(self):


        fig, ax = plt.subplots()

        # set fig size
        fig.set_size_inches(10, 10)

        ax.set_xlim(-0.05, 0.1)
        ax.set_ylim(-0.05, 0.1)
        ax.set_aspect('equal')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')

        position_history = np.array([[0.0, 0.0]])
        scat = ax.scatter(position_history[:, 0], position_history[:, 1], s = 100)
        
        def update(interval):
            nonlocal position_history

            print('updating visualizatino')
            transformations = self.s.latest
            head_pos = transformations["head"][:, :3, 3]
            print(head_pos[:, 0])
            new_pos = np.array([[head_pos[0, 0], -head_pos[0, 1]]])  # Adjust axis if necessary
            # print(new_pos.shape)
            position_history = np.append(position_history, new_pos, axis=0)
            # scat.set_offsets(np.c_[head_pos[:, 0], - head_pos[:, 2]])
            scat.set_offsets(position_history)

            xmin = min(position_history[:, 0])
            xmax = max(position_history[:, 0])
            ymin = min(position_history[:, 1])
            ymax = max(position_history[:, 1])

            ax.set_xlim(xmin - 0.1, xmax + 0.1)
            ax.set_ylim(ymin - 0.1, ymax + 0.1)

            # make x, y axis text size larger
            for item in ([ax.title, ax.xaxis.label, ax.yaxis.label] +
                        ax.get_xticklabels() + ax.get_yticklabels()):
                item.set_fontsize(20)

        ani = animation.FuncAnimation(fig, update, frames=10000, interval=1)
        plt.show()

    def run(self):
            
        thread = Thread(target=self.background_localization_visualization)
        thread.start()

    
if __name__ == "__main__":

    import argparse 
    import os 

    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', type = str, required = True)
    parser.add_argument('--record', action = 'store_true')
    args = parser.parse_args()

    env = LocalizationVisualizer(args)

    env.run()
