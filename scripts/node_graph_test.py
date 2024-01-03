from PySide2 import QtCore, QtGui, QtWidgets
from PySide2.QtCore import Qt
from PySide2.QtGui import QIcon
from PySide2.QtWidgets import QApplication, QWidget, QHBoxLayout, QLabel, QSlider, QGridLayout
import signal

import sys
from PySide2 import QtCore, QtGui, QtWidgets
import pydot
from pydot import Dot, Edge 

from ament_index_python.packages import get_package_share_directory

from pprint import pprint
import time
from NodeGraphQt import (
    NodeGraph,
    BaseNode,
    PropertiesBinWidget,
    NodesTreeWidget,
    NodesPaletteWidget
)
from NodeGraphQt.constants import (
    URI_SCHEME,
    URN_SCHEME,
    LayoutDirectionEnum,
    PipeLayoutEnum,
    PortTypeEnum,
    ViewerEnum
)

import os

node_graph_wrapper = None

import numpy as np

import glob, os
from pprint import pprint
import networkx as nx
import matplotlib.pyplot as plt
from networkx import drawing

class NodeGraphWrapper: 



    def __init__(self):

        self.graph = NodeGraph()
        self.graph.register_nodes([ROSNode])
        
        start = time.time()
        self.load_bv()
        # load_with_networkx(self, filename):

        end = time.time()
        print("Total time to load bv:", end-start)
        self.organize_graph()
        # self.setup_graph_utils()

    def setup_graph_utils(self):

        package_share_directory = get_package_share_directory('ros_bard')
        print("total path :", os.path.join(package_share_directory,'config/hotkeys/hotkeys.json' ))
        self.graph.set_context_menu_from_file( os.path.join(package_share_directory,'config/hotkeys/hotkeys.json' ))


    def organize_graph(self):
        # self.graph.set_layout_direction(1)
        # self.graph.set_pipe_style(style=2)
        self.graph.set_pipe_collision(True)
        self.graph.set_pipe_slicing(False)
        
        # self.graph.auto_layout_nodes()
        self.graph.select_all()
        self.graph.fit_to_selection()

        self.graph.clear_selection()

    def clear_graph(self):
        all_nodes = self.graph.all_nodes()
        self.graph.delete_nodes(all_nodes)

    def load_with_networkx(self, filename):
        if not os.path.isfile(filename):
            raise ValueError
        G = drawing.nx_agraph.read_dot(filename)
        pos = nx.planar_layout(G)

        node_objects = {}

        for node in G.nodes:
            # print("node position: ", )
            ros_node = self.graph.create_node(
                'nodes.basic.ROSNode', 
                name=node, 
                color='#0a1e20', 
                pos = (pos[node]*np.array((1100, 800))).tolist()
                # pos=()).tolist()
                )
            # print(type(G.out_edges(node)._data("label")))
            registered_ports = []
            for src, dest, data in G.out_edges(node,data=True):
                # print(ros_node.output_ports)
                if data["label"] not in registered_ports:
                    ros_node.add_output(data["label"])
                    registered_ports.append(data["label"])
                
            registered_ports = []
            for src, dest, data in G.in_edges(node,data=True):
                # print(ros_node.output_ports)
                if data["label"] not in registered_ports:
                    ros_node.add_input(data["label"])
                    registered_ports.append(data["label"])

            node_objects[node] = ros_node            


        for src, dest, data in G.edges(data = True):
            if src in node_objects and dest in node_objects:
                for port_num, output in enumerate(node_objects[src]._outputs):
                    input_names = [ obj.name() for obj in node_objects[dest]._inputs]

                    if output.name() in input_names :
                        in_port_indx = input_names.index(output.name())
                        node_objects[src].set_output(port_num, node_objects[dest]._inputs[in_port_indx] )

        self.organize_graph()

    def load_bv(self, filename=None):
        if filename is None: 
            filename = "/home/misha/code/ros_bard_ws/src/ros_bard/data/2023-12-28_13:52:52.593381/2023-12-28_13:52:53.687049.bv"
        
        with open(filename, 'r') as file:
            data = file.read()

        graphs = pydot.graph_from_dot_data( data )
        dot_graph = graphs[0]

        nodes = {}
        for edge in dot_graph.get_edges():
            if edge.get_source() not in nodes: 
                nodes[edge.get_source()] = {"pubs":[], "subs":[] }
            if edge.get_destination() not in nodes:
                nodes[edge.get_destination()] = {"pubs":[], "subs":[] }

            nodes[edge.get_source()]["pubs"].append(edge.get_attributes()["label"])
            nodes[edge.get_destination()]["subs"].append(edge.get_attributes()["label"])

        node_objects = {}
        for node, pub_sub in nodes.items(): 
            ros_node = self.graph.create_node(
                'nodes.basic.ROSNode', 
                name=node, 
                color='#0a1e20')
            for pub in set(pub_sub["pubs"]):
                ros_node.add_output(pub)
            for sub in set(pub_sub["subs"]):
                ros_node.add_input(sub)
            node_objects[node] = ros_node

        for edge in dot_graph.get_edges():
            for port_num, output in enumerate(node_objects[edge.get_source()]._outputs):
                input_names = [ obj.name() for obj in node_objects[edge.get_destination()]._inputs]

                if output.name() in input_names :
                    in_port_indx = input_names.index(output.name())
                    node_objects[edge.get_source()].set_output(port_num, node_objects[edge.get_destination()]._inputs[in_port_indx] )

        self.organize_graph()

class ROSNode(BaseNode):
    # unique node identifier.
    __identifier__ = 'nodes.basic'
    # initial default node name.
    NODE_NAME = 'node A'

    def __init__(self):
        super(ROSNode, self).__init__()


class Slider(QWidget):
    
    files = []

    def __init__(self, directory=None):
        super().__init__()
 
        self.createSlider(directory)
        # self.setIcon()
        # self.show()
 
    def setIcon(self):
        appIcon = QIcon("icon.png")
        self.setWindowIcon(appIcon)
 
 
    def createSlider(self, directory=None):
        hbox = QHBoxLayout()
 
        self.slider = QSlider()
        self.slider.setOrientation(Qt.Horizontal)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.setTickInterval(10)

        if directory is not None: 
            os.chdir(directory)
            for file in glob.glob("*.bv"):
                self.files.append(os.path.join(directory, file))

            self.slider.setMinimum(0)
            self.slider.setMaximum(len(self.files))        
        else:
            self.slider.setMinimum(0)
            self.slider.setMaximum(100)

        self.slider.valueChanged.connect(self.changedValue)
 
        self.label = QLabel("0")
        self.label.setFont(QtGui.QFont("Ubuntu Mono Regular", 11))
 
        hbox.addWidget(self.slider)
        hbox.addWidget(self.label)
 
        self.setLayout(hbox)
 
 
    def changedValue(self):
        size = self.slider.value()
        selected_file = self.files[size]
        global node_graph_wrapper
        node_graph_wrapper.clear_graph()
        # node_graph_wrapper.load_bv(selected_file)
        node_graph_wrapper.load_with_networkx(selected_file)

        self.label.setText(str(size))
 
class Window(QWidget):

    def __init__(self):

        super().__init__()

        self.setWindowTitle("QGridLayout Example")

        layout = QGridLayout()
        global node_graph_wrapper
        node_graph_wrapper = NodeGraphWrapper()

        layout.addWidget(node_graph_wrapper.graph.widget, 1, 0)

        layout.addWidget(Slider("/home/misha/code/ros_bard_ws/src/ros_bard/data/2023-12-28_13:53:21.487343"), 0, 0)

        self.resize(1100, 800)
        self.setLayout(layout)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)

    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.show()
    app.exec_()
