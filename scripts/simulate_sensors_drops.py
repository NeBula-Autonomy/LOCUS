 
"""
Description:  
    - Simulate sensors drops  
Author: 
    - Matteo Palieri (palierimatteo@gmail.com)
"""

import rosbag 

robot_name = "husky4"
data_path = "/home/snuc/Downloads/Datasets/urban/beta_2/husky4/rosbag/"

odom_topic = None
imu_topic = "/" + robot_name + "/vn100/imu_wori_wcov"

if "spot" in robot_name: 
    odom_topic = "/" + robot_name + "/visual_odom"
elif "husky" in robot_name: 
    odom_topic = "/" + robot_name + "/hero/wio_ekf/odom"

in_odom_bag  = rosbag.Bag(data_path + "husky4_state_2020-02-26-12-51-09_0.bag")
in_imu_bag   = rosbag.Bag(data_path + "husky4_state_tcp_no_delay_2020-02-26-12-51-09_0.bag")
out_odom_bag = rosbag.Bag(data_path + "husky4_wio_drop.bag", "w")
out_imu_bag  = rosbag.Bag(data_path + "husky4_imu_drop.bag", "w")

odom_health = [[0,650], [740,780]]
imu_health  = [[0,680], [710,770]]
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