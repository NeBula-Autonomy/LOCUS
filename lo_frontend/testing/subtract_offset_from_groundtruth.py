#!/usr/bin/env python
import rospy
import math 
import numpy as np 
from geometry_msgs.msg import PoseStamped, Vector3, Vector3Stamped
from scipy.spatial.transform import Rotation as R

# TODO: Publish also the very first message !

firstMessageReceived = False 
v_R_l0, v_t_l0 = None, None 

firstCfMessageReceived = False 
v_R_l0Cf = None
initial_rpy_x, initial_rpy_y, initial_rpy_z = 0, 0, 0 

pub = rospy.Publisher("/Robot_7/pose/no_offset", PoseStamped)
pub_rpy = rospy.Publisher("/Robot_7/pose/rpy/no_offset", Vector3)

pub_rpyCf = rospy.Publisher("/imu/rpy/filtered/no_offset", Vector3)

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

# imu/rpy/filtered is the output topic of IMU Complementary Filter 
# geometry_msgs/Vector3Stamped are the type of messages transported on this topic 
# Here we already have roll pitch yaw
def ComplementaryFilterCallback(data):

    global firstCfMessageReceived, initial_rpy_x, initial_rpy_y, initial_rpy_z

    if (firstCfMessageReceived==False):

        initial_rpy_x = data.vector.x
        initial_rpy_y = data.vector.y
        initial_rpy_z = data.vector.z
        firstCfMessageReceived = True
    
    else: 
        new_message = Vector3()
        print("data.vector.y" + str(data.vector.y))
        print("initial_rpy_y" + str(initial_rpy_y))
        new_message.x = data.vector.x - initial_rpy_x
        new_message.y = data.vector.y - initial_rpy_y
        new_message.z = data.vector.z - initial_rpy_z
        pub_rpyCf.publish(new_message)

def subtract_offset_to_gt():
    rospy.init_node('subtract_offset_from_gt', anonymous=True)
    rospy.Subscriber("/Robot_7/pose", PoseStamped, GroundTruthCallback)
    rospy.Subscriber("/imu/rpy/filtered", Vector3Stamped, ComplementaryFilterCallback)
    rospy.spin()

if __name__ == '__main__':
    try: 
        subtract_offset_to_gt()
    except rospy.ROSInterruptException:
        pass