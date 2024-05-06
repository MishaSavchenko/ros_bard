from ros_bard.LocalUIInterface import LocalUIInterface

if __name__ == "__main__":
    ui_interface = LocalUIInterface()
    ui_interface.window.show()
    ui_interface.app.exec()