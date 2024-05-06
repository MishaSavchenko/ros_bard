import sys

from PySide6 import QtCore, QtGui, QtWidgets
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
        self.window = loader.load("/home/misha/code/ros_bard_ws/untitled.ui", None)
        self.find_objects_of_interst(self.window)
        self.setup_events()

    def find_objects_of_interst(self, ui_object):
        for child in ui_object.children():
            if child.objectName() in self.elements_of_interest.keys(): 
                    self.elements_of_interest[child.objectName()] = child 
            self.find_objects_of_interst(child)

    def setup_events(self):
        self.elements_of_interest["next"].clicked.connect(self.data_interface.next)
        self.elements_of_interest["next"].clicked.connect(self.update_data)
        
        self.elements_of_interest["previous"].clicked.connect(self.data_interface.prev)
        self.elements_of_interest["previous"].clicked.connect(self.update_data)

    def update_data(self):
        previous_title_text, \
        previous_text, \
        current_title_text, \
        current_text = self.data_interface.get_data()
        
        self.elements_of_interest["current_title"].setText(current_title_text)
        self.elements_of_interest["last_title"].setText(previous_title_text)

        self.elements_of_interest["current_status"].setText(current_text)
        self.elements_of_interest["last_status"].setText(previous_text)


    def run(self):
        self.window.show()
        self.app.exec()
