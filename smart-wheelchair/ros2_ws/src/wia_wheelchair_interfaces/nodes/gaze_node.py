#!/usr/bin/env python3
"""
WIA Smart Wheelchair - Eye Gaze Control Node Wrapper

ROS2 node wrapper for eye gaze controller integration.
"""

import sys
import os

# Add integrations to path
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', '..', '..', '..', 'integrations', 'eye-gaze'
))

from gaze_controller import GazeControllerNode, main

if __name__ == '__main__':
    main()
