import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(os.path.dirname(HERE))
for pkg in ('carbot_common', 'carbot_ops', 'carbot_control', 'uwb_localization'):
    sys.path.insert(0, os.path.join(SRC, pkg))
