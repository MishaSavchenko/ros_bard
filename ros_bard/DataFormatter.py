from copy import deepcopy
from PySide6.QtWidgets import QTreeWidgetItem
from PySide6 import QtGui
import re

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

class CustomColors(Enum):
    RED = QtGui.QColor(239, 91, 91, 255)
    GREEN = QtGui.QColor(113, 247, 159, 255)
    YELLOW = QtGui.QColor(240, 201, 135, 255)
    GRAY = QtGui.QColor(125, 132, 145, 255)

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

    @staticmethod
    def paint_tree_data(previous_tree, current_tree, diff):
        if "dictionary_item_removed" in diff.keys():
            for removed in diff["dictionary_item_removed"]:
                try:
                    node_name = re.search("root\[\'(.*)\'\]", removed).group(1)
                    for tree_node in previous_tree:
                        if tree_node.text(0) == node_name:
                            DataFormatter.paint_node(tree_node, CustomColors.RED.value)
                except AttributeError:
                    continue

        if "dictionary_item_added" in diff.keys():
            for removed in diff["dictionary_item_added"]:
                try:
                    node_name = re.search("root\[\'(.*)\'\]", removed).group(1)
                    for tree_node in current_tree:
                        if tree_node.text(0) == node_name:
                            DataFormatter.paint_node(tree_node, CustomColors.GREEN.value)
                except AttributeError:
                    continue

    @staticmethod
    def paint_node(node, color, columns=2):
        for i in range(2):    
            if node.columnCount() != columns and node.childCount() == 0:
                node.setForeground(i, CustomColors.GRAY.value)
            else: 
                node.setForeground(i, color)

        for j in range(node.childCount()):
            DataFormatter.paint_node(node.child(j), color)

