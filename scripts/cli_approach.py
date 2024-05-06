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

def main(frequency=30, 
         include_hidden=True, 
         verbose=False):

    current_time = datetime.now()
    formatted_time = current_time.strftime('%Y%m%d%H%M%S')
    save_file = open("/home/misha/code/ros_bard_ws/src/ros_bard/data/yaml_files/"+formatted_time+".json", "a")
    last_time_stamp = 0
    try:
        with NodeStrategy(None) as node:
            while True:
                time_stamp = datetime.now().timestamp()
                # data[time_stamp] = {}
                new_data = {}
                start_time = time.time()

                node_names = get_node_names(
                    node=node, include_hidden_nodes=include_hidden)

                for n in node_names:
                    try: 
                        new_data[n.full_name] = get_node_data(node, n.full_name, time_stamp)
                    except xmlrpc.client.Fault as e:
                        print(e)
                        continue

                end_time = time.time()
                if last_time_stamp in data.keys():
                    diff = DeepDiff(new_data, 
                                    data[last_time_stamp], 
                                    ignore_order=True, 
                                    report_repetition=True)
                    if len(diff) != 0: 
                        data[time_stamp] = new_data
                        last_time_stamp = time_stamp
                else: 
                    data[time_stamp] = new_data
                    last_time_stamp = time_stamp

                if verbose:
                    print("Loop time :", end_time - start_time)
                time.sleep(1/frequency)
                
    except KeyboardInterrupt:
        json.dump(data, save_file)
        pass

if __name__ == "__main__":
    main(frequency=30,
         include_hidden=True)