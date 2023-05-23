#!/usr/bin/env python3
import rosbag
from std_msgs.msg import Int32, String

bag = rosbag.Bag('/home/mavis/catkin_ws/src/hri_preferences_study/src/rosbags/test.bag', 'w')

try:
    s = String()
    s.data = 'foo'

    i = Int32()
    i.data = 42 

    bag.write('chatter', s)
    bag.write('numbers', i)

finally:
    bag.close()