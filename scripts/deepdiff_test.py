import deepdiff
from deepdiff import DeepDiff, Delta


import json
from pprint import pprint

def main(): 
    with open("/home/misha/code/ros_bard_ws/src/ros_bard/data/json_files/20240505154744.json", "r") as file:
        data = json.load(file)
    
    time_stamps = list(data.keys())
    time_stamps.sort()

    a = data[time_stamps[0]]
    b = data[time_stamps[199]]

    diff = DeepDiff(a, b, ignore_order=True, report_repetition=True)
    
    a + Delta(diff) == b # Should be true
    b - Delta(diff, bidirectional=True) == a # Should be true
    # return 

if __name__ == "__main__":
    main()