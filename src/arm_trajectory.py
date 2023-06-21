#!/usr/bin/env python3

import kinova_msgs.srv 
import rospy

class Trajectory:

    def __init__(self):    
        self.positions = [] 

    def _add_pose(self, data):
        self.positions.append([data.X, data.Y, data.Z, data.ThetaX, data.ThetaY, data.ThetaZ])

    # Record the arm's trajectory as it moves
    def record(self):
        print("recording")
        self.recorder = rospy.Subscriber("/j2s7s300_driver/out/cartesian_command", kinova_msgs.msg.KinovaPose, self._add_pose) 

    def stop_recording(self):
        print("Stopping recording")
        self.recorder.unregister()

    # play back the trajectory in reverse
    def execute_reverse_trajectory(self):

        rospy.wait_for_service("/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory")
        add_service = rospy.ServiceProxy("/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory", kinova_msgs.srv.AddPoseToCartesianTrajectory)
        
        for pos in reversed(self.positions):
            add_service(*pos)
