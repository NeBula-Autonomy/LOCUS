"""
Description:  
    - A general-purpose sensor health monitor 
Author: 
    - Matteo Palieri (palierimatteo@gmail.com)
"""



import rospy
import threading 
from sensor_msgs.msg import PointCloud2



def sensor_timeout(id):
    print("Detected failure in sensor " + str(id))



def sensor_callback(msg, sensor_id):
    global timers 
    timers[sensor_id].cancel()
    timers[sensor_id] = threading.Timer(timeout_threshold, sensor_timeout, args=(sensor_id,)) 
    timers[sensor_id].start()



rospy.init_node("sensors_health_monitor")



topics = ["/husky4/velodyne_points/transformed", 
          "/husky4/velodyne_rear/velodyne_points/transformed",
          "/husky4/velodyne_front/velodyne_points/transformed"]



timers = []
timeout_threshold = 3

for i in range(len(topics)): 
    rospy.Subscriber(topics[i], PointCloud2, sensor_callback, i)
    timers.append(threading.Timer(timeout_threshold, sensor_timeout, args=(i,)))



for timer in timers: 
    timer.start()



while not rospy.is_shutdown():
    rospy.sleep(1)
