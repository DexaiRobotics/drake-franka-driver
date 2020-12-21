#! /usr/bin/env python3

import subprocess
import time

import click
import lcm
from robot_msgs import bool_t


class FrankaDriverServer:
    def __init__(self, robot_name):

        self.robot_name = robot_name
        self.driver_running = False

        self.lcm = lcm.LCM("udpm://239.255.76.67:7667?ttl=2")
        self.lcm.subscribe(
            robot_name + "_START_DRIVER", self.handle_start_driver_request
        )
        self.lcm.subscribe(
            robot_name + "_STOP_DRIVER", self.handle_stop_driver_request
        )

        self.driver_running_channel = robot_name + "_DRIVER_RUNNING"

    def handle_start_driver_request(self, channel, req):
        print("Starting driver!")
        cmd = "cd /src/drake-franka-driver && ./franka.sh"
        print(cmd)
        subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)

    def handle_stop_driver_request(self, channel, req):
        print("Stopping driver!")
        cmd = "pkill -f franka_driver"
        print(cmd)
        subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)

    def run_server(self):
        print("Starting Franka Driver Server")

    def run(self):
        self.lcm.handle_timeout(0.005)

    def publish_state(self):
        driver_running_msg = bool_t()
        driver_running_msg.utime = time.time() * 1e6
        driver_running_msg.data = self.driver_running
        self.lcm.publish(
            self.driver_running_channel, driver_running_msg.encode()
        )


@click.command()
@click.argument("robot_name", required=True)
def cli(robot_name):
    """Run the server for CLI invocation."""
    fds = FrankaDriverServer(robot_name)
    fds.run_server()
    try:
        while True:
            fds.run()
            time.sleep(1.0 / 30.0)
    except KeyboardInterrupt:
        print("interrupted!")


if __name__ == "__main__":
    cli()  # pylint: disable=E1120
