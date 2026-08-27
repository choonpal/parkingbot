import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/guitest/parkingbot/ros2/install_runtime/cooperative_parking_robot'
