#! /usr/bin/env python2

from __future__ import print_function
import time
from robot_msgs import bool_t
import sys
import lcm
import subprocess


def get_current_utime():
    time_now = time.time()
    utime = time_now * 1e6
    return utime

class FrankaDriverServer:
    def __init__(self, robot_name):

        self.robot_name = robot_name
        self.driver_running = False

        self.lcm = lcm.LCM('udpm://239.255.76.67:7667?ttl=2')
        actuate_tool_sub = self.lcm.subscribe(robot_name+'_START_DRIVER', self.handle_start_driver_request)
        open_tool_sub    = self.lcm.subscribe(robot_name+'_STOP_DRIVER',  self.handle_stop_driver_request)

        self.driver_running_channel = robot_name+'_DRIVER_RUNNING'


    def handle_start_driver_request(self, channel, req):
        print("Starting driver!")
        cmd = 'cd /src/drake-franka-driver && ./franka_0.sh'
        print(cmd)
        subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)

    def handle_stop_driver_request(self, channel, req):
        print("Stopping driver!")
        cmd = 'pkill -f franka_plan_runner'
        print(cmd)
        subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)

    def run_server(self):
        print("Starting Franka Driver Server")

    def run(self):
        self.lcm.handle_timeout(0.005)


    def publish_state(self):
        driver_running_msg = bool_t()
        driver_running_msg.utime = get_current_utime()
        driver_running_msg.data = self.driver_running
        self.lcm.publish(self.driver_running_channel, driver_running_msg.encode())

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print('Usage: ./franka_server.py <ROBOT_NAME>')
        exit(0)

    robot_name = sys.argv[1]

    rps = FrankaDriverServer(robot_name)
    rps.run_server()
    try:
        while True:
            rps.run()
#            rps.publish_state()
            time.sleep(1.0 / 30.0)
    except KeyboardInterrupt:
        print('interrupted!')