#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import *
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from object_recognition_msgs.msg import *
from geometry_msgs.msg import *
import tf

def callback(data):


def object_pose_transform():

    rospy.init_node('object_pose_transform', anonymous=True)
    
    rospy.Subscriber("object_to_pick", Pose, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
	object_pose_transform()
    listener = tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/object_frame', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    except rospy.ROSInterruptException:
        pass
