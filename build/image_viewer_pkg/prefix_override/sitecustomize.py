import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/drone_dev/ros_ws/install/image_viewer_pkg'
