 
"""
Description:  
    - Simulate sensors drops  
Author: 
    - Matteo Palieri (palierimatteo@gmail.com)
"""

import rosbag 

odom_topic = "/spot2/visual_odom"
imu_topic = "/spot2/vn100/imu_wori_wcov"

in_odom_bag  = rosbag.Bag("spot2_state_2021-08-05-14-04-51_0.bag")
in_imu_bag   = rosbag.Bag("spot2_state_tcp_no_delay_2021-08-05-14-04-48_0.bag")
out_odom_bag = rosbag.Bag("spot2_visual_odom_drop.bag", "w")
out_imu_bag  = rosbag.Bag("spot2_imu_drop.bag", "w")

odom_health = [[0,230], [260,290], [320,350]]
imu_health  = [[0,290], [340,370]]
first_odom_time = None 
first_imu_time  = None 

for topic, msg, t in in_odom_bag.read_messages(topics=[odom_topic]):
    if first_odom_time == None: 
        first_odom_time = t 
    elapsed_time_odom = (t - first_odom_time).to_sec()
    for el in odom_health: 
        if el[0] < elapsed_time_odom < el[1]: 
            out_odom_bag.write(odom_topic, msg, t)

for topic, msg, t in in_imu_bag.read_messages(topics=[imu_topic]):
    if first_imu_time == None: 
        first_imu_time = t 
    elapsed_time_imu = (t - first_imu_time).to_sec()
    for el in imu_health: 
        if el[0] < elapsed_time_imu < el[1]: 
            out_imu_bag.write(imu_topic, msg, t)

out_odom_bag.close()
out_imu_bag.close()