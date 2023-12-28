import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pprint import pprint
import time
import graphviz

import numpy as np 
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

from ament_index_python.packages import get_package_share_directory

from itertools import product

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')


def main(args=None):


    # may raise PackageNotFoundError
    package_share_directory = get_package_share_directory('my_package_name')
    storage_directory = "/data"

    # g.view()
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    node_info = {}

    adjacency_matrix = []
    adjacency_matrix = np.zeros((1, 1, 3))

    topic_info = {}


    while True: 


        g = graphviz.Digraph('G', filename='hello.gv')
        g.attr('graph', 
               layout="sfdp", 
               beautify="false",
               voro_margin="1.0",
               repulsiveforce="10.0")
        # g.attr('graph')  # use dot
        
        g.attr('node', shape="box")
        total_edges = {}


        topics = minimal_publisher.get_topic_names_and_types()
        for topic in topics: 
            if "action" in topic[0] or "parameter_event" in topic[0]:
                continue

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
        # input("Waiting for you m'lord")
        time.sleep(1.0)
            
            # print("-----------------")
            # for ti in pub_info + sub_info: 
            #     print(ti)
        # pprint(topics)
        # quit()
        # break


    # 0: topic
    # 1: service 
    # 2: action

    # while True: 
    #     # namespaces = node_names_namespaces[:,1]
    #     # topic_names_and_types = minimal_publisher.get_topic_names_and_types()
    #     # pprint(topic_names_and_types)

    #     node_names_namespaces = minimal_publisher.get_node_names_and_namespaces()
    #     node_names = [nnn[0] for nnn in node_names_namespaces]
    #     namespaces = [nnn[1] for nnn in node_names_namespaces]

    #     adjacency_matrix.resize(len(node_names), len(node_names), 3)

    #     for indx, node_name in enumerate(node_names):
    #         # print("--------------------------------------")

    #         publishers = minimal_publisher.get_publisher_names_and_types_by_node(node_names[indx], 
    #                                                                              namespaces[indx])
            
    #         subscribers = minimal_publisher.get_subscriber_names_and_types_by_node(node_names[indx], 
    #                                                                                namespaces[indx])




    #         if node_name not in node_info:
    #             node_info[node_name] = {"pub":{"name": None, "type":None}, "sub":{"name": None, "type":None}}

    #         node_info[node_name]["pub"]["name"] = [p[0] for p in publishers]
    #         node_info[node_name]["pub"]["type"] = [p[1][0] for p in publishers]

    #         node_info[node_name]["sub"]["name"] = [s[0] for s in subscribers]
    #         node_info[node_name]["sub"]["type"] = [s[1][0] for s in subscribers]

    #     break

    # adjacency_matrix = np.zeros((len(node_info), len(node_info), 3))
    # for node_name, v in node_info.items():
    #     pubs = v["pub"]["name"]
    #     subs = v["sub"]["name"]
    #     print(pubs)
    #     print(subs)
    #     # print(node_name)
    #     # print(v)
    # # print(adjacency_matrix)
    # # pprint(node_info)

    quit()
    g = graphviz.Graph('G', filename='hello.gv')
    g.attr('node', shape="box")

    g.render()
    time.sleep(0.25)
    
    rclpy.spin_once(minimal_publisher)
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()