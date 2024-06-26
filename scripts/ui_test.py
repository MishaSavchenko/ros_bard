#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory

from ros_bard.LocalUIInterface import LocalUIInterface

if __name__ == "__main__":
    ros_bard_path = get_package_share_directory('ros_bard')
    ui_layout_path = os.path.join(ros_bard_path, "config", "tree_layout.ui")

    ui_interface = LocalUIInterface(ui_layout_path)
    ui_interface.run()