#!/usr/bin/env python



"""
Description:  
    - A general-purpose sensor health monitor 
Author: 
    - Matteo Palieri (palierimatteo@gmail.com)
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
    dead_sensors.append(sensor_id)



def sensor_callback(msg, sensor_id):
    global timers, first_msg_received, resurrection_detection_pub
    if not first_msg_received: 
        for timer in timers: 
            timer.start()
        first_msg_received = True 
    if sensor_id in dead_sensors: 
        dead_sensors.remove(sensor_id)
        resurrection_msg = Int8()
        resurrection_msg.data = sensor_id
        resurrection_detection_pub.publish(resurrection_msg)
    timers[sensor_id].cancel()
    timers[sensor_id] = threading.Timer(timeout_threshold, sensor_timeout, args=(sensor_id,)) 
    timers[sensor_id].start()



rospy.init_node("sensors_health_monitor")
robot_name = rospy.get_namespace().split('/')[1]
failure_detection_pub = rospy.Publisher("failure_detection", Int8, queue_size=1)
resurrection_detection_pub = rospy.Publisher("resurrection_detection", Int8, queue_size=1)



first_msg_received = False 
dead_sensors = []
timers = []
timeout_threshold = 1



number_of_velodynes = rospy.get_param("/" + robot_name + "/point_cloud_merger_lo/merging/number_of_velodynes")
topics = ["velodyne_points/transformed", 
          "velodyne_front/velodyne_points/transformed"]
if number_of_velodynes == 3: 
    topics.append("velodyne_rear/velodyne_points/transformed")



for i in range(len(topics)): 
    rospy.Subscriber("/" + robot_name + "/" + topics[i], PointCloud2, sensor_callback, i)
    timers.append(threading.Timer(timeout_threshold, sensor_timeout, args=(i,)))



while not rospy.is_shutdown():
    rospy.sleep(1)
