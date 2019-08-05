#! /usr/bin/env python2

from __future__ import print_function
import time
from robot_msgs import bool_t
import sys
import lcm


def get_current_utime():
    time_now = time.time()
    utime = time_now * 1e6
    return utime


if __name__ == '__main__':
    lcm = lcm.LCM('udpm://239.255.76.67:7667?ttl=2')

    if len(sys.argv) < 2:
        print('Usage: ./publish_bool.py <LCM_CHANNEL> <0/1>')
        exit(0)

    lcm_channel = sys.argv[1]
    bool_msg = bool_t()

    if len(sys.argv) == 3 and sys.argv[2] == '0':
        bool_msg.data = False
    else:
        bool_msg.data = True

    
    bool_msg.utime = get_current_utime()
    lcm.publish(lcm_channel, bool_msg.encode())


