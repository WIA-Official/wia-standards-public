#!/usr/bin/env python3
"""
WIA Smart Wheelchair - Exoskeleton Transition Node Wrapper

ROS2 node wrapper for exoskeleton transition manager integration.
"""

import sys
import os

# Add integrations to path
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', '..', '..', '..', 'integrations', 'exoskeleton'
))

from transition_manager import TransitionManagerNode, main

if __name__ == '__main__':
    main()
