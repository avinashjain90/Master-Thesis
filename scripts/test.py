#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import *
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from object_recognition_msgs.msg import *
from geometry_msgs.msg import *

objects = RecognizedObjectArray()
#objects = RecognizedObject()
ObjectPose = Pose()
object_to_pick = PoseWithCovarianceStamped() 

def callback(data):
    confidence_list = []
    object_type = list()
    object_pose = list()
    mydict = {}
    
    for recObj in data.objects:
    	objects = recObj
#    	confidence_list = objects.confidence
#    	object_type = objects.ObjectType
#    	object_pose =objects.pose
    
    	#mydict[objects.type.key] = {objects.type.key, objects.pose, objects.confidence}
    	mydict[objects.type.key] = {'key':objects.type.key, 'pose':objects.pose, 'confidence': objects.confidence}

    highConfidence = 0
    #print mydict
#    objtype = ObjectType()
    print "confi"
    for key,value in mydict.items():

    	print value['confidence']
	if value['confidence'] > highConfidence:
	    highConfidence = value['key']
	    #objtype = key

    if highConfidence!= 0:
    	object_to_pick = mydict[highConfidence]['pose']
    print object_to_pick
    ObjectPose = object_to_pick.pose.pose
    pub = rospy.Publisher('object_to_pick',Pose, queue_size=10)
    pub.publish(ObjectPose)
    #rate.sleep()
   # print "high"
   # print highConfidence

def object_pose():

    rospy.init_node('object_pose', anonymous=True)
    
    rospy.Subscriber("recognized_object_array", RecognizedObjectArray, callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
	object_pose()
        
    except rospy.ROSInterruptException:
        pass

