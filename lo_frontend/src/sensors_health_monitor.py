#!/usr/bin/env python



"""
Description:  
    - A general-purpose sensor health monitor 
Author: 
    - Matteo Palieri (palierimatteo@gmail.com)
TODO: 
    - Add startup handling 
"""



import rospy
import threading 
from std_msgs.msg import Int8
from sensor_msgs.msg import PointCloud2



def sensor_timeout(sensor_id):
    global failure_detection_pub
    failure_msg = Int8()
    failure_msg.data = sensor_id
    failure_detection_pub.publish(failure_msg)



def sensor_callback(msg, sensor_id):
    global timers 
    timers[sensor_id].cancel()
    timers[sensor_id] = threading.Timer(timeout_threshold, sensor_timeout, args=(sensor_id,)) 
    timers[sensor_id].start()



rospy.init_node("sensors_health_monitor")
robot_name = rospy.get_namespace().split('/')[1]
failure_detection_pub = rospy.Publisher("failure_detection", Int8, queue_size=1)



timers = []
timeout_threshold = 10
topics = ["velodyne_points/transformed", 
          "velodyne_front/velodyne_points/transformed", 
          "velodyne_rear/velodyne_points/transformed"]



for i in range(len(topics)): 
    rospy.Subscriber("/" + robot_name + "/" + topics[i], PointCloud2, sensor_callback, i)
    timers.append(threading.Timer(timeout_threshold, sensor_timeout, args=(i,)))



for timer in timers: 
    timer.start()



while not rospy.is_shutdown():
    rospy.sleep(1)
