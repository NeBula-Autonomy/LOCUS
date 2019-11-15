#!/usr/bin/env python
import rospy
import math 
import numpy as np 
from geometry_msgs.msg import PoseStamped, Vector3, Vector3Stamped
from scipy.spatial.transform import Rotation as R

# TODO: Publish also the very first message !

# INPUT TOPIC 
groundtruth_topic = "/Robot_7/pose"

# OUTPUT TOPIC 
position_topic = "/Robot_7/pose/no_offset"
orientation_topic = "/Robot_7/pose/rpy/no_offset"

firstMessageReceived = False 
v_R_l0, v_t_l0 = None, None 

pub = rospy.Publisher(position_topic, PoseStamped)
pub_rpy = rospy.Publisher(orientation_topic, Vector3)

def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
def rotationMatrixToEulerAngles(R) : 
    assert(isRotationMatrix(R))     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])     
    singular = sy < 1e-6 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0 
    return np.array([x, y, z])

def GroundTruthCallback(data):

    global firstMessageReceived, v_R_l0, v_t_l0 

    if (firstMessageReceived==False):
        initial_pos_x = data.pose.position.x
        initial_pos_y = data.pose.position.y
        initial_pos_z = data.pose.position.z
        initial_quat_x = data.pose.orientation.x
        initial_quat_y = data.pose.orientation.y
        initial_quat_z = data.pose.orientation.z
        initial_quat_w = data.pose.orientation.w        
        v_R_l0 = R.from_quat([initial_quat_x, initial_quat_y, initial_quat_z, initial_quat_w])
        v_t_l0 = [initial_pos_x, initial_pos_y, initial_pos_z]
        firstMessageReceived = True
    
    else: 
        v_R_lk = R.from_quat([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        v_t_lk = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        v_R_lk_array = v_R_lk.as_dcm()
        v_R_l0_array = v_R_l0.as_dcm()
        output_translation = np.dot(np.transpose(v_R_l0_array), (np.array(v_t_lk)-np.array(v_t_l0)))
        new_message = PoseStamped()
        new_message.pose.position.x = output_translation[0] 
        new_message.pose.position.y = output_translation[1] 
        new_message.pose.position.z = output_translation[2] 
        pub.publish(new_message)         
        output_rotation = np.dot(np.transpose(v_R_l0_array), v_R_lk_array)
        output_rpy = rotationMatrixToEulerAngles(output_rotation)
        new_message = Vector3()
        new_message.x = output_rpy[0]
        new_message.y = output_rpy[1]
        new_message.z = output_rpy[2]
        pub_rpy.publish(new_message)

def subtract_offset_to_gt():
    rospy.init_node('subtract_offset_from_gt', anonymous=True)
    rospy.Subscriber(groundtruth_topic, PoseStamped, GroundTruthCallback)
    rospy.spin()

if __name__ == '__main__':
    try: 
        subtract_offset_to_gt()
    except rospy.ROSInterruptException:
        pass