#!/usr/bin/env python3
import os
import time
import graphviz
from datetime import datetime
from itertools import product

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

def main(args=None):

    # may raise PackageNotFoundError
    package_share_directory = get_package_share_directory('ros_bard')
    package_share_directory = "/home/misha/code/ros_bard_ws/src/ros_bard"
    storage_directory = "data"
    data_path = os.path.join(package_share_directory, storage_directory)
    
    if not os.path.exists(data_path):
        raise OSError
        quit()

    print("Creating a directory in data...")

    # Create a new directory to store log files 
    mode = 0o755
    new_dir_name = str(datetime.now()).replace(" ", "_")
    path = os.path.join(data_path, new_dir_name) 
    os.mkdir(path, mode) 
    logging_path = path

    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    node_info = {}
    
    while True:  
        new_file_name = str(datetime.now()).replace(" ", "_") + ".bv"
        new_file = os.path.join(logging_path, new_file_name)

        g = graphviz.Digraph('G', filename=new_file)
        g.attr('graph', 
               layout="sfdp", 
               beautify="false",
               voro_margin="1.0",
               repulsiveforce="10.0")
        
        g.attr('node', shape="box")
        total_edges = {}

        topics = minimal_publisher.get_topic_names_and_types()
        for topic in topics: 
            # if "action" in topic[0] or "parameter_event" in topic[0]:
            #     continue

            pub_info = minimal_publisher._get_info_by_topic(topic[0], False, _rclpy.rclpy_get_publishers_info_by_topic)
            sub_info = minimal_publisher._get_info_by_topic(topic[0], False, _rclpy.rclpy_get_subscriptions_info_by_topic)


            pub_nodes = [pi.node_name for pi in pub_info ]
            sub_nodes = [si.node_name for si in sub_info ]

            edges = list(product(pub_nodes, sub_nodes))

            for edge in edges:

                edge_definition = tuple(list(edge) + [topic[0]]) 
                if edge_definition in total_edges: 
                    continue
                else: 
                    total_edges[edge_definition] = 0
                    g.edge(edge_definition[0], edge_definition[1], label=edge_definition[2])

        g.render()
        time.sleep(1.0)

if __name__ == '__main__':
    main()