import json
from pprint import pprint

from ros_bard.DataFormatter import DataFormatter

def main(): 
    with open("/home/misha/code/ros_bard_ws/src/ros_bard/data/json_files/20240210102622.json", "r") as file:
        data = json.load(file)
    
    df = DataFormatter(remove_hidden_nodes=False)

    for timestamp, node_data in data.items():
        for node, data in node_data.items():
            formatted_data = df.format_node(data)
            print('===============================================================')
            print(formatted_data)

if __name__ == "__main__":
    main()
