#!/usr/bin/env python3
"""
WIA Smart Wheelchair - Voice Control Node Wrapper

ROS2 node wrapper for voice command controller integration.
"""

import sys
import os

# Add integrations to path
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', '..', '..', '..', 'integrations', 'voice'
))

from voice_controller import VoiceControllerNode, main

if __name__ == '__main__':
    main()
