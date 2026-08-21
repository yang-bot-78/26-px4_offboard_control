import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robot/egohx_ws/26-px4_offboard_control-main/install/offboard_nav2_planning'
