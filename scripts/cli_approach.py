from ros2node.verb import VerbExtension
from ros2node.api import NodeNameCompleter
from ros2node.api import INFO_NONUNIQUE_WARNING_TEMPLATE
from ros2action.api import get_action_names
from ros2action.api import get_action_clients_and_servers

import xmlrpc
from ros2cli.node.strategy import NodeStrategy

from ros2node.api import get_node_names
from ros2node.api import get_publisher_info
from ros2node.api import get_subscriber_info
from ros2node.api import get_action_client_info
from ros2node.api import get_action_server_info
from ros2node.api import get_service_client_info
from ros2node.api import get_service_server_info

import sys
import time
import json
from pprint import pprint
from deepdiff import DeepDiff
from datetime import datetime

def print_names_and_types(names_and_types):
    print(*[2 * '  ' + s.name + ': ' + ', '.join(s.types)
          for s in names_and_types], sep='\n')

def convert_to_print(names_and_types):
    return [(s.name, s.types) for s in names_and_types]
    
def get_node_data(node, remote_node_name, include_hidden=True):
    subscribers = get_subscriber_info(node=node,
                                remote_node_name=remote_node_name,
                                include_hidden=include_hidden)

    publishers = get_publisher_info(node=node, 
                                remote_node_name=remote_node_name, 
                                include_hidden=include_hidden)

    service_servers = get_service_server_info(node=node, 
                                        remote_node_name=remote_node_name, 
                                        include_hidden=include_hidden)

    service_clients = get_service_client_info(node=node, 
                                        remote_node_name=remote_node_name, 
                                        include_hidden=include_hidden)

    actions_servers = get_action_server_info(node=node, 
                                        remote_node_name=remote_node_name, 
                                        include_hidden=include_hidden)
    actions_clients = get_action_client_info(node=node, 
                                        remote_node_name=remote_node_name, 
                                        include_hidden=include_hidden)
    

    return {"name": remote_node_name,
            "sub": convert_to_print(subscribers),
            "pub": convert_to_print(publishers),
            "srv_services": convert_to_print(service_servers), 
            "srv_clients": convert_to_print(service_clients),
            "act_services": convert_to_print(actions_servers), 
            "act_clients": convert_to_print(actions_clients)
            }

data = {}

def main(frequency=10, 
         include_hidden=True, 
         verbose=False):

    save_file = open("/home/misha/code/ros_bard_ws/src/ros_bard/data/test.txt", "a")
    try:
        with NodeStrategy(None) as node:
            while True:
                time_stamp = datetime.now().timestamp()
                data[time_stamp] = {}
                start_time = time.time()

                node_names = get_node_names(
                    node=node, include_hidden_nodes=include_hidden)

                for n in node_names:
                    try: 
                        data[time_stamp][n.full_name] = get_node_data(node, n.full_name, time_stamp)
                    except xmlrpc.client.Fault as e:
                        print(e)
                        continue
                    # if verbose: 
                    #     print(remote_node_name)
                    #     print('  Subscribers:')
                    #     print_names_and_types(subscribers)
                    #     print('  Publishers:')
                    #     print_names_and_types(publishers)
                    #     print('  Service Servers:')
                    #     print_names_and_types(service_servers)
                    #     print('  Service Clients:')
                    #     print_names_and_types(service_clients)
                    #     print('  Action Servers:')
                    #     print_names_and_types(actions_servers)
                    #     print('  Action Clients:')
                    #     print_names_and_types(actions_clients)

                    # pprint(data[time_stamp])
                end_time = time.time()
                # with open('person_data.pkl', 'wb') as fp:

                if verbose:
                    print("Loop time :", end_time - start_time)
                time.sleep(1/frequency)
    except KeyboardInterrupt:
        # print("keyboard interupt")
        json.dump(data, save_file)
        pass

if __name__ == "__main__":
    main(frequency=10, 
         include_hidden=True)