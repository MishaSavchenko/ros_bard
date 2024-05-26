import sys

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtWidgets import QHeaderView
from PySide6.QtUiTools import QUiLoader

from ros_bard.LocalDataInterface import LocalDataInterface

class LocalUIInterface:
    data_interface: LocalDataInterface = None

    elements_of_interest = {
                            "current_title":None,
                            "current_status": None,
                            "last_title":None,
                            "last_status": None,
                            "previous": None,
                            "next": None,
                            }

    def __init__(self):
        self.data_interface  = LocalDataInterface()
        # test_file = "/home/misha/code/ros_bard_ws/src/ros_bard/data/json_files/20240210102622.json"
        # test_file = "/home/misha/code/ros_bard_ws/src/ros_bard/data/json_files/20240210102018.json"
        test_file = "/home/misha/code/ros_bard_ws/src/ros_bard/data/json_files/20240210102505.json"
        self.data_interface.load_data(test_file)

        loader = QUiLoader()
        self.app = QtWidgets.QApplication(sys.argv)
        # self.window = loader.load("/home/misha/code/ros_bard_ws/untitled.ui", None)
        self.window = loader.load("/home/misha/code/ros_bard_ws/tree_and_text_layout.ui", None)
        
        self.find_objects_of_interst(self.window)

        self.setup_style()
        self.setup_events()
        self.update_data()

    def find_objects_of_interst(self, ui_object):
        for child in ui_object.children():
            if child.objectName() in self.elements_of_interest.keys(): 
                    self.elements_of_interest[child.objectName()] = child 
            self.find_objects_of_interst(child)

    def setup_style(self):
        self.elements_of_interest["current_status"].setHeaderLabels(["Name", "Type"])
        self.elements_of_interest["current_status"].header().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.elements_of_interest["current_status"].header().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)

        self.elements_of_interest["last_status"].setHeaderLabels(["Name", "Type"])
        self.elements_of_interest["last_status"].header().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.elements_of_interest["last_status"].header().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)


    def setup_events(self):
        self.elements_of_interest["next"].clicked.connect(self.data_interface.next)
        self.elements_of_interest["next"].clicked.connect(self.update_data)
        
        self.elements_of_interest["previous"].clicked.connect(self.data_interface.prev)
        self.elements_of_interest["previous"].clicked.connect(self.update_data)

    def update_data(self):
        previous_title_text, \
        previous_data, \
        current_title_text, \
        current_data = self.data_interface.get_data()
        
        self.elements_of_interest["last_status"].clear()
        self.elements_of_interest["last_status"].addTopLevelItems(previous_data)
        
        self.elements_of_interest["current_status"].clear()
        self.elements_of_interest["current_status"].addTopLevelItems(current_data)
        
        self.elements_of_interest["current_title"].setText(current_title_text)
        self.elements_of_interest["last_title"].setText(previous_title_text)

    def run(self):
        self.window.show()
        self.app.exec()
