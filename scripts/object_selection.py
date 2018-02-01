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
from tf import transformations as t
from tf.transformations import euler_from_quaternion
objects = RecognizedObjectArray()
#objects = RecognizedObject()
ObjectPose = Pose()
object_to_pick = PoseWithCovarianceStamped() 
notpicked = 1

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
    #print "confi"
    for key,value in mydict.items():

    	# print value['confidence']
	if value['confidence'] > highConfidence:
	    highConfidence = value['key']
	    #objtype = key

    # ObjectPose = []
    if highConfidence!= 0 and notpicked == 1:
        global notpicked
        notpicked = 0
    	object_to_pick = mydict[highConfidence]['pose']
        #print object_to_pick
        global ObjectPose
    	ObjectPose = object_to_pick.pose.pose

    # print ObjectPose
	pub = rospy.Publisher('object_to_pick',Pose, queue_size=10)
	pub.publish(ObjectPose)
    print ObjectPose
    #rate.sleep()
   # print "high"
   # print highConfidence
    # ObjectPose = ObjectPose.inverse()
    br = tf.TransformBroadcaster()
    
    
    explicit_trans = [ObjectPose.position.x, ObjectPose.position.y, ObjectPose.position.z]

    explicit_quat = [ObjectPose.orientation.x, ObjectPose.orientation.y, ObjectPose.orientation.z, ObjectPose.orientation.w]

    #your_euler = t.euler_from_quaternion(explicit_quat)

    # t.translation_matrix(explicit_trans)
 
    transform = t.concatenate_matrices(t.translation_matrix(explicit_trans), t.quaternion_matrix(explicit_quat))
    inversed_transform = t.inverse_matrix(transform)
    translation = t.translation_from_matrix(inversed_transform)
    quaternion = t.quaternion_from_matrix(inversed_transform)
    print translation
    br.sendTransform((translation[0], translation[1], translation[2]),
                      (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
                     # (inversed_transform.orientation.x, inversed_transform.orientation.y, inversed_transform.orientation.z, inversed_transform.orientation.w),
                       rospy.Time.now(), "object_frame", "kinect2_link")
   # br.sendTransform((explicit_trans[0], explicit_trans[1], explicit_trans[2]),
       #               (explicit_quat[0], explicit_quat[1], explicit_quat[2], explicit_quat[3]),
                     # (inversed_transform.orientation.x, inversed_transform.orientation.y, inversed_transform.orientation.z, inversed_transform.orientation.w),
#                       rospy.Time.now(), "object_frame", "kinect2_link")

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

