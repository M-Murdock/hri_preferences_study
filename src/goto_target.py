#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from kinova_msgs.msg import KinovaPose 
from kinova_msgs.srv import AddPoseToCartesianTrajectory

#  def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

if __name__ == '__main__':
    try:
        # talker()
        pose_service = rospy.ServiceProxy('/kinova_msgs/AddPoseToCartesianTrajectory', KinovaPose)
    except rospy.ROSInterruptException:
        pass