import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(os.path.dirname(HERE))
for pkg in ('carbot_common', 'carbot_detectors', 'carbot_perception'):
    sys.path.insert(0, os.path.join(SRC, pkg))
