import json
import yaml
from copy import deepcopy
from pprint import pprint, pformat
from datetime import datetime
from enum import Enum

from ros_bard.DataFormatter import DataFormatter 

from deepdiff import DeepDiff

class LocalDataInterface: 
    
    previous_title_text: str = ""
    previous_text: str = ""

    next_title_text: str = ""
    next_text: str = ""

    index: int = 0

    data_container = []

    formatter: DataFormatter = None

    def __init__(self):
        self.formatter = DataFormatter()

    def load_data(self, file):
        with open(file, "r") as file:
            res = json.loads(file.read())    
            self.parse_data(res)
            return res
    
    def parse_data(self, data):
        ordered_timestamps = sorted(data.keys(), key=lambda ts: float(ts))

        for ots in ordered_timestamps: 

            entry = data[ots]
            entry["timestamp"] = float(ots)

            self.data_container.append(entry)

    def at(self, indx):
        if indx >= len(self.data_container): 
            raise IndexError

        self.index = indx

        prev_data_point = deepcopy(self.data_container[indx - 1])
        curr_data_point = deepcopy(self.data_container[indx])

        self.previous_title_text = str(datetime.fromtimestamp(prev_data_point.pop("timestamp")))

        self.current_title_text = str(datetime.fromtimestamp(curr_data_point.pop("timestamp")))

        self.previous_text = self.formatter.format(prev_data_point)
        self.current_text = self.formatter.format(curr_data_point)

        diff = DeepDiff(prev_data_point, curr_data_point, ignore_order=True, report_repetition=True)
        self.formatter.paint_tree_data(self.previous_text, self.current_text, diff)

        return self.previous_title_text, \
                self.previous_text, \
                self.current_title_text, \
                self.current_text

    def get_data(self):
        return self.at(self.index)

    def next(self):
        self.index = (self.index + 1) % len(self.data_container)
        print("new index :", self.index)

    def prev(self):
        self.index = (self.index - 1) % len(self.data_container)
        print("new index :",self.index)
