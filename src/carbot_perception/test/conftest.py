import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
# source-tree runs without colcon: make carbot_common importable
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), 'carbot_common'))
