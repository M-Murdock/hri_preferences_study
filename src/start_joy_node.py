#!/usr/bin/env python3

import subprocess
import os
import signal

class JoyNodeLauncher:
    def __init__(self):
        self.joy_process = None

    def start_joy_node(self):
        if self.joy_process is None:
            print("Starting joy node...")
            # Launch joy node using rosrun
            self.joy_process = subprocess.Popen(
                ['rosrun', 'joy', 'joy_node'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # So we can terminate the whole process group later
            )
        else:
            print("Joy node is already running.")

    def stop_joy_node(self):
        if self.joy_process is not None:
            print("Stopping joy node...")
            os.killpg(os.getpgid(self.joy_process.pid), signal.SIGTERM)
            self.joy_process = None
        else:
            print("Joy node is not running.")


