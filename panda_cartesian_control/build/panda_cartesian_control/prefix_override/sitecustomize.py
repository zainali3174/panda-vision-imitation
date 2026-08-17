import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/developer/multipanda_ws/src/panda_cartesian_control/install/panda_cartesian_control'
