import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(os.path.dirname(HERE))
# source-tree runs without colcon: make the sibling packages importable
for pkg in ('carbot_common', 'carbot_planning'):
    sys.path.insert(0, os.path.join(SRC, pkg))
sys.path.insert(0, HERE)
