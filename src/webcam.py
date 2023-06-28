#!/usr/bin/env python3

# Saves webcam video
import numpy as np
import sys
import cv2
import os
from threading import Thread


class Webcam:
    def __init__(self, user_id):
        self.user_id = user_id
        self.running = False

    def start(self):
        Thread(target=self.get, args=()).start()
        return self 

    def get(self):
        self.running = True
        # This will return video from the first webcam on your computer.
        self.cap = cv2.VideoCapture(0)  
        
        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')

        path = "/" + str(self.user_id) + '.avi'
        self.out = cv2.VideoWriter(path, fourcc, 20.0, (640, 480))
        
        # loop runs if capturing has been initialized. 
        while(self.running):
            # reads frames from a camera 
            # ret checks return at each frame
            ret, frame = self.cap.read() 
            
            # # output the frame
            self.out.write(frame) 

            # The original input frame is shown in the window 
            cv2.imshow('Original', frame)
        


        # Close the window / Release webcam
        self.cap.release()
        
        # After we release our webcam, we also release the output
        self.out.release() 
        
        # De-allocate any associated memory usage 
        cv2.destroyAllWindows()

    def quit(self):
        self.running = False
