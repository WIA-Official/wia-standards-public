#!/usr/bin/env python3
"""
WIA Smart Wheelchair - Smart Home Bridge Node Wrapper

ROS2 node wrapper for Matter smart home bridge integration.
"""

import sys
import os

# Add integrations to path
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', '..', '..', '..', 'integrations', 'smarthome'
))

from matter_bridge import MatterBridgeNode, main

if __name__ == '__main__':
    main()
