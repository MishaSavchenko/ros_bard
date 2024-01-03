from pprint import pprint
import networkx as nx
import matplotlib.pyplot as plt
from networkx import drawing

import time
if __name__ == "__main__":
    filename = "/home/misha/code/ros_bard_ws/src/ros_bard/data/2023-12-28_13:53:21.487343/2023-12-28_13:53:21.549300.bv"
    start_time = time.time()

    G = drawing.nx_agraph.read_dot(filename)
    pos = nx.planar_layout(G)

    end_time = time.time()
    total_time = end_time - start_time

    print("Total time to load and calculate positions :", total_time)
    nx.draw(G, pos=pos, with_labels=True)
    plt.draw()  
    plt.show()