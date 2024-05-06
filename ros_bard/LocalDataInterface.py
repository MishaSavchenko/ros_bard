import json
import yaml
from copy import deepcopy
from pprint import pprint, pformat
from datetime import datetime

class LocalDataInterface: 
    
    previous_title_text: str = ""
    previous_text: str = ""

    next_title_text: str = ""
    next_text: str = ""

    index: int = 0

    data_container = []

    def __init__(self):
        pass

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

    def at(self, indx, json_format=True):
        if indx >= len(self.data_container): 
            raise IndexError

        self.index = indx

        prev_data_point = deepcopy(self.data_container[indx - 1])
        curr_data_point = deepcopy(self.data_container[indx])

        self.previous_title_text = str(datetime.fromtimestamp(prev_data_point.pop("timestamp")))

        self.current_title_text = str(datetime.fromtimestamp(curr_data_point.pop("timestamp")))

        if json_format:
            self.previous_text = pformat(prev_data_point, indent=4)
            self.current_text = pformat(curr_data_point, indent=4)

        else: 
            self.previous_text =yaml.dump(prev_data_point, sort_keys=False)
            self.current_text = yaml.dump(curr_data_point, sort_keys=False)

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
