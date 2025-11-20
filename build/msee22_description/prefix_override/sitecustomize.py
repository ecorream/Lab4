import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/min/a/ecorream/ece569-fall2025/Lab4/install/msee22_description'
