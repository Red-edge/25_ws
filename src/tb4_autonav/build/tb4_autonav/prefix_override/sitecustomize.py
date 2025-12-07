import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tony/25_ws/src/tb4_autonav/install/tb4_autonav'
