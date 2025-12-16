#!/usr/bin/env python3
"""
WIA Smart Wheelchair - BCI Control Node Wrapper

ROS2 node wrapper for brain-computer interface controller integration.
"""

import sys
import os

# Add integrations to path
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', '..', '..', '..', 'integrations', 'bci'
))

from bci_controller import BCIControllerNode, main

if __name__ == '__main__':
    main()
