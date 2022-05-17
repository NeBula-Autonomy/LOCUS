"""
Description:  
    - Aggregates user requested odometries along with ground truth odometry 
      in aggregated_odometries.bag
Author: 
    - Matteo Palieri, NASA Jet Propulsion Laboratory
    - Andrzej Reinke, NASA Jet Propulsion Laboratory
"""

import os
import sys
import rosbag
import rospkg
import yaml

def main():

    methods = []

    # if len(sys.argv)<5:
    #     print("Minimal Usage: python aggregate_odometries.py settings_filename")
    #     print("Example Usage: python aggregate_odometries.py robot_name method1 method2 method3 method4")
    #     sys.exit(1)  
    if len(sys.argv)<3:
        print("Example Usage: python aggregate_odometries.py data_dir robot_name ground_truth_topic run_number")
        sys.exit(1)  

    data_dir = sys.argv[1]
    robot_name = sys.argv[2]
    ground_truth_topic = sys.argv[3]
    run_number = sys.argv[4]
    params = {}

    settings_filename = rospkg.RosPack().get_path('gt_analysis') + "/config/analyzer_settings.yaml"

    params = {}

    with open(settings_filename) as file:
        params = yaml.load(file, Loader=yaml.FullLoader)



    results_dir = data_dir + "/results_" + run_number + "/"
    if not os.path.exists(results_dir):
        os.mkdir(results_dir)

    os.chdir(results_dir)

    default_case = not params["specify_non_default_methods"]

    methods.append("ground_truth")

    if not default_case:
        for i in range(len(params["specific_methods"])):
            methods.append(params["specific_methods"][i])
    else:
        # methods.append("lamp")
        methods.append("locus")

    d = {}

    for method in methods: 
        if method == "ground_truth":
            bag_name = data_dir + "/" + method + "/odometry.bag"
            topic_name = ground_truth_topic
        else:
            bag_name = data_dir + "/" + method + "_" + run_number + "/odometry.bag"
            topic_name = "/" + robot_name + "/lo_frontend/odometry"
        d[method] = [bag_name, topic_name]
    # import pdb ; pdb.set_trace()
    with rosbag.Bag("aggregated_odometries.bag", "w") as outbag: 
        for key in d.keys():
            print("Aggregating " + key + " ...")
            bag = rosbag.Bag(d[key][0])
            if key != "ground_truth":
                d[key][1] =  list(bag.get_type_and_topic_info()[1].keys())[0]
            print("Topic for " + key + " odometry is " + d[key][1] + "\n")
            for topic, msg, t in bag.read_messages(topics=[d[key][1]]):
                outbag.write(key, msg, t)
    outbag.close()

if __name__=="__main__":
    main()
