#! /usr/bin/env python3
"""Script to publish a boolean msg to an LCM channel."""


import sys
import time

import lcm
from robot_msgs import bool_t

if __name__ == "__main__":
    lcm = lcm.LCM("udpm://239.255.76.67:7667?ttl=2")

    if len(sys.argv) < 2:
        print("Usage: ./publish_bool.py <LCM_CHANNEL> <0/1>")
        sys.exit(1)

    lcm_channel = sys.argv[1]
    bool_msg = bool_t()

    if len(sys.argv) == 3 and sys.argv[2] == "0":
        bool_msg.data = False
    else:
        bool_msg.data = True

    bool_msg.utime = time.time() * 1e6
    lcm.publish(lcm_channel, bool_msg.encode())
