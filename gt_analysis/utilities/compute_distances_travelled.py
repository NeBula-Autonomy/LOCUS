"""
Description:  
    - INPUT: 
        - aggregated_odometries.bag 
    - OUTPUT: 
        - aggregated_odometries_100.bag
        - aggregated_odometries_200.bag
        - aggregated_odometries_300.bag
Author: 
    - Matteo Palieri, NASA Jet Propulsion Laboratory
"""

import sys
import rosbag 
import numpy as np 

def main():

    if len(sys.argv)<2:
        print("Minimal Usage: python segment_aggregated_odometries.py method1")
        print("Example Usage: python segment_aggregated_odometries.py method1 method2 method3 method4")
        sys.exit(1)  

    methods = []    
    for i in range(len(sys.argv)):
        if i!=0:
            methods.append(sys.argv[i]) 
        
        
    outbag = rosbag.Bag("aggregated_odometries_" + str(checkpoint) + ".bag", "w")

    initialized = False
    time_end_point = None
    distance_traveled = []
    previous_position = None  
    
    for topic, msg, t in rosbag.Bag("aggregated_odometries.bag").read_messages(topics=["ground_truth"]):
        
        if not initialized: 
            previous_position = msg.pose.pose.position 
            distance_traveled.append(0)
            initialized = True
            pass
        
        current_position = msg.pose.pose.position  
        dx = np.square(current_position.x - previous_position.x)
        dy = np.square(current_position.y - previous_position.y) 
        dz = np.square(current_position.z - previous_position.z)
        distance_traveled = distance_traveled + np.sqrt(dx + dy + dz) 
        previous_position = current_position
        
        if distance_traveled>checkpoint: 
            time_end_point = t
            break

    for method in methods:  
        for topic, msg, t in rosbag.Bag("aggregated_odometries.bag").read_messages(topics=[method]):
            outbag.write(method, msg , t)
            if t>time_end_point: 
                break
    
    outbag.close() 

if __name__ == "__main__": 
    main()