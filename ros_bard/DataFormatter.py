from copy import deepcopy
from PySide6.QtWidgets import QTreeWidgetItem
from enum import Enum

NAME_PARSER = {
    'act_clients': "Action Clients",
    'act_services': "Action Services",
    'pub': "Publishers",
    'srv_clients': "Server Clients",
    'srv_services': "Servers Services",
    'sub': "Subscribers",
    'name': "Name"
}

FORMAT_TEMPLATE = "{:<10} {:<50} {:>40}\n"

class ReturnType(Enum):
    STRING = 0
    TREE = 1
    # JSON = 2
    # YAML = 3

class DataFormatter:

    _remove_hidden_nodes: bool 
    _format: ReturnType
    def __init__(self,
                 remove_hidden_nodes:bool = True, 
                 format:ReturnType = ReturnType.TREE):
        self._remove_hidden_nodes = remove_hidden_nodes
        self._format = format

    def format(self, data: dict):
        if self._format == ReturnType.STRING: 
            return self.to_string(data)
        elif self._format == ReturnType.TREE:
            return self.to_tree(data)
        else: 
            raise TypeError
    
    def to_string(self, data: dict):
        res_data = deepcopy(data)
        
        if self._remove_hidden_nodes:
            for node in list(res_data.keys()): 
                if node[:2] == '/_':
                    del res_data[node]

        output_str = ""
        output_str += FORMAT_TEMPLATE.format(" "," "," ")
        
        for k,v in res_data.items():
            output_str += self.format_node(v)
            output_str += "---"
        
        return output_str

    def format_node(self, node:dict):
        output_str = ""
        output_str += FORMAT_TEMPLATE.format(" "," "," ")
        # print(node)
        # input()
        for data_type, data in list(node.items()):
            if data_type == "name":
                output_str += FORMAT_TEMPLATE.format(NAME_PARSER[data_type], " ", " ")
                output_str += FORMAT_TEMPLATE.format(" ", data, " ")
            else:
                output_str += FORMAT_TEMPLATE.format(NAME_PARSER[data_type], " ", " ")
                output_str += self.format_interface_data(data)
        return output_str
    @staticmethod
    def to_tree(data_point):
        tree_nodes = []

        for node_name, node in data_point.items():
            tree_node = QTreeWidgetItem((node_name, ))
            for data_type, data in node.items():
                data_node = QTreeWidgetItem((NAME_PARSER[data_type],))
                if data_type == "name":
                    continue
                for d in data:
                    data_node.addChild( QTreeWidgetItem((d[0], d[1][0]) ) )
                tree_node.addChild(data_node)
            tree_nodes.append(tree_node)
        
        return tree_nodes

    @staticmethod
    def format_interface_data(data):
        output_str = ""
        if isinstance(data, list):
            for d in data:
                output_str += FORMAT_TEMPLATE.format(" ", d[0], d[1][0])
            return output_str
        return output_str
