#!/usr/bin/env python3

# launches a gui which will emergency stop the kinova arm

import rospy
import numpy as np
import sys
import tkinter 
import study_runner
import os
import kinova_msgs.srv

class EstopFrame(tkinter.Frame):

    def __init__(self, root):
        super().__init__(root)

        self._root = root
        self.grid(sticky="NSEW")
        root.rowconfigure(0, weight=1)
        root.columnconfigure(0, weight=1)

        # set up bottom frame with buttons
        self.start_frame = tkinter.Frame(self)
        self.start_frame.grid(row=1, column=0, columnspan=2, sticky='esw', padx=2, pady=2)


        self.home_button = tkinter.Button(self.start_frame,
                                           text="Home", bg = "yellow", height= 10, width=20,
                                           command=self._home_button_callback)
        self.home_button.grid(row=0, column=2, sticky="EW")

    def _home_button_callback(self):
        rospy.wait_for_service("/j2s7s300_driver/in/home_arm")
        home_arm = rospy.ServiceProxy("/j2s7s300_driver/in/home_arm", kinova_msgs.srv.HomeArm)
        home_arm()

if __name__ == "__main__":
    try: 
        rospy.init_node("home", anonymous=True)

        root = tkinter.Tk()
        root.geometry("200x200+300+600")
        home = EstopFrame(root)
        study_runner.runner.main(home)

        rospy.spin()
    except:
        pass