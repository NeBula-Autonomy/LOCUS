"""
Description:  
    - Utilities used by analyzer.py
Author: 
    - Matteo Palieri, NASA Jet Propulsion Laboratory
    - Andrzej Reinke, NASA Jet Propulsion Laboratory
"""

import os
import yaml
import math 
import rospy
import rosbag
import zipfile
import matplotlib 
import numpy as np
import seaborn as sns 
import matplotlib.cm as cm
import scipy.stats as stats
from nav_msgs.msg import Odometry 
from matplotlib.lines import Line2D
from matplotlib import pyplot as plt 
from loop_closure_eval import LoopClosureEvaluator
import key_handling
from functools import reduce
matplotlib.rcParams['mathtext.fontset'] = 'stix'
matplotlib.rcParams['font.family'] = 'STIXGeneral'
size, labelsize = 50, 60 
params = {'legend.fontsize': 'large',
          'figure.figsize': (20,8),
          'axes.labelsize': labelsize,
          'axes.titlesize': labelsize,
          'xtick.labelsize': size*0.75,
          'ytick.labelsize': size*0.75,
          'axes.titlepad': 25}
plt.rcParams.update(params)

methods = []
color_mapping = {0:"red", 1:"green", 2:"blue", 3:"yellow", 4:"pink", 5:"black"} 
  
def enable_dark_mode():
    global plt 
    plt.style.use("dark_background")

def enable_full_screen():
    global plt 
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())

def get_avg(data): 
    return reduce(lambda x,y: x+y, data)/len(data) 

def evaluate_normal_distribution(data): 
    distributions = [[],[],[],[],[]]
    for i in range(len(data)):
        for el in data[i]:
            if len(el) != 0:
                min = np.min(el)
                max = np.max(el)
                avg = get_avg(el)
                var = np.var(el)
                distributions[i].append([min, max, avg, var])
            else: 
                distributions[i].append([0, 0, 0, 0])    
    return distributions

def visualize_raw_data(title, units, data, results_dir, show=False, fullscreen=False):  
    fig = plt.figure()  
    for i in range(len(data)):  
        if i<len(color_mapping):
            plt.plot(data[i], label=methods[i], color=color_mapping[i], linewidth=3)
            plt.xlabel("Time")
            plt.ylabel(units)
            plt.legend(prop={'size': 30})
        else: 
            print("Update color_mapping")
    if fullscreen:
        enable_full_screen()
    plt.tight_layout()
    fig.savefig(results_dir + "/plots/" + title)
    plt.title(title)
    if show:
        plt.show()
    plt.close()

def save_txt(filename, data): 
    outfile = open(filename, "w")
    outfile.write("Min: " + str(data[0]) + "\n" + "Max: " + str(data[1]) + "\n" + "Avg: " + str(data[2]) + "\n" + "Var: " + str(data[3]) + "\n")
    outfile.close()

def visualize_and_save_normal_distributions(all_results, results_dir, show=False, fullscreen=False):  
    for key in all_results: 
        fig = plt.figure() 
        data = all_results[key]
        for el in data[0]: 
            title = data[1]
            plt.plot(el[0], el[1], label=el[2], linewidth=3)
            plt.xlabel(title)
            plt.ylabel("Count")
            plt.legend(prop={'size': 30})
        if fullscreen:
            enable_full_screen()
        plt.tight_layout()
        fig.savefig(results_dir + "/plots/" + title)
        if show:
            plt.show()
        plt.close() 

def extract_last_msg_from_bag(bag):
    last_msg = None
    for topic, msg, t in bag.read_messages():
        last_msg = msg
    return last_msg

robot_to_prefix = {"husky1" : 'a', "husky2" : 'b', "husky3" : 'c', "husky4" : 'f', "spot1" : 'g', "spot2" : 'h'}

def pose_graph_to_odometry(lamp_data_path, robot_name):
    pose_graph_bag = rosbag.Bag(lamp_data_path + "pose_graph.bag")
    out_topic = "/" + robot_name + "/lamp/odometry"
    pose_graph = extract_last_msg_from_bag(pose_graph_bag)
    if pose_graph is not None: 
        with rosbag.Bag(lamp_data_path + "odometry.bag", "w") as odometry_bag: 
            for node in pose_graph.nodes:
                if node.ID=="odom_node":
                    pfx, idx = key_handling.split_pg_key(node.key)
                    if pfx == robot_to_prefix[robot_name]:
                        msg = Odometry()
                        msg.header.stamp = node.header.stamp
                        msg.pose.pose = node.pose 
                        odometry_bag.write(out_topic, msg, msg.header.stamp)
        odometry_bag.close()
    else: 
        print("\n-------------------------------------------------------")
        print("WARNING Unable to retrieve pose_graph from pose_graph.bag")
        print("-------------------------------------------------------\n")


def plot_volume_size(volume, size, timestamps, results_dir, show=False, fullscreen=False): 
    fig, ax1 = plt.subplots()
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('Volume (m^3)', color='tab:red')
    ax1.plot(timestamps, volume, color='tab:red', linewidth=3)
    ax1.tick_params(axis='y', labelcolor='tab:red')
    ax2 = ax1.twinx()  
    ax2.set_ylabel('Number of points in map', color='tab:blue')  
    ax2.plot(timestamps, size, color='tab:blue', linewidth=3)
    ax2.tick_params(axis='y', labelcolor='tab:blue')
    fig.tight_layout()  
    fig.savefig(results_dir + "/plots/VolumevsSize")
    if fullscreen:
        enable_full_screen()
    if show:
        plt.show()
    plt.close()

def segment_aggregated_odometries(methods, checkpoints):    
    for checkpoint in checkpoints:          
        outbag = rosbag.Bag("aggregated_odometries_" + str(checkpoint) + ".bag", "w")
        initialized = False
        time_end_point = None
        distance_traveled = 0
        previous_position = None          
        for topic, msg, t in rosbag.Bag("aggregated_odometries.bag").read_messages(topics=["ground_truth"]):          
            outbag.write("ground_truth", msg , t)  
            if not initialized: 
                previous_position = msg.pose.pose.position 
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

def boxplot_from_npz(methods, checkpoints, results_dir, show=False, fullscreen=False): 
    data = []
    for distance in checkpoints: 
        for method in methods: 
            filename = os.getcwd() + "/ape_results_" + str(distance) + "/" + method +"/error_array.npy"
            data.append(np.load(filename))
        data.append([])  
    bplot = sns.boxplot(data=[d for d in data], width=0.5)   
    colors = ["red", "green", "blue", "yellow", "gray", "black"]    
    color_counter = 0
    for i in range(len(data)-len(checkpoints)): 
        mybox = bplot.artists[i]
        mybox.set_facecolor(colors[color_counter])
        color_counter = color_counter + 1
        if color_counter == len(methods): 
            color_counter=0  
    bplot.set(xlabel= " Distance traveled [m] ", ylabel='APE [m]') 
    bplot.set_xticks(range((len(methods)*len(checkpoints)) + len(checkpoints))) 
    bplot.set_xticklabels([" " for c in checkpoints]) # TODO: xtick checkpoints
    legend_elements = []
    for i in range(len(methods)): 
        legend_elements.append(Line2D([0], [0], color=colors[i], lw=4, label=methods[i]),)
    leg = plt.legend(handles=legend_elements, loc='upper_left', prop={'size': 30})
    for legobj in leg.legendHandles:
        legobj.set_linewidth(10.0)
    plt.rcParams.update(params)
    plt.title('APE over distances')
    if fullscreen:
        enable_full_screen()
    if show:
        plt.show()
    plt.close()

def make_evo_options_string(ape_type, method, b_align_origin, b_show, b_save):

    out_string = ""

    if b_align_origin:
        out_string += " --align_origin"
    
    if b_show:
        out_string += " --plot"

    if b_save:        
        if ape_type == "single":
            out_string += " --save_plot"
            out_string += " plots/" + method + "_ape_plots.pdf"
        elif ape_type == "combined":
            out_string += " --save_plot"
            out_string += " plots/combined_ape_plots.pdf"
            out_string += " --save_table"
            out_string += " combined_ape.csv"
        elif ape_type == "traj":
            out_string += " --save_plot"
            out_string += " plots/traj_plots.pdf"
        elif ape_type == "trans":
            out_string += " --save_plot"
            out_string += " plots/trans_plots_" + method + ".pdf"
        elif ape_type == "rot":
            out_string += " --save_plot"
            out_string += " plots/rot_plots_" + method + ".pdf"
        elif ape_type == "combined_trans":
            out_string += " --save_plot"
            out_string += " plots/trans_plots.pdf"
            out_string += " --save_table"
            out_string += " trans_ape.csv"
        elif ape_type == "combined_rot":
            out_string += " --save_plot"
            out_string += " plots/rot_plots.pdf"
            out_string += " --save_table"
            out_string += " rot_ape.csv"
        else:
            print("Error - first input should be 'single' or 'combined'")

    return out_string


def compute_distance_based_error_metrics(methods, results_dir, show=False, fullscreen=False, min_move=1.0): 
    ape_trans = {}
    ape_rot = {}
    timestamps = {}
    initialized = {}
    time_end_point = {}
    distance_traveled = {}
    distance_time = {}
    previous_position = {}

    for method in methods: 
        filename = results_dir + "ape_results/" + method +"_trans/error_array.npy"
        ape_trans[method] = np.load(filename)
        filename = results_dir + "ape_results/" + method +"_rot/error_array.npy"
        ape_rot[method] = np.load(filename)
        filename = results_dir + "ape_results/" + method +"_trans/timestamps.npy"
        timestamps[method] = np.load(filename)
        initialized[method] = False
        distance_traveled[method] = []
        previous_position[method] = None
        distance_time[method] = []

    # Add ground truth?
    initialized['ground_truth'] = False
    distance_traveled['ground_truth'] = []
    previous_position['ground_truth'] = None
    distance_time['ground_truth'] = []
    
    for topic, msg, t in rosbag.Bag(results_dir + "aggregated_odometries.bag").read_messages():          
        
        if topic in methods or topic == 'ground_truth':
            distance_time[topic].append(msg.header.stamp.to_sec())
            if not initialized[topic]:
                previous_position[topic] = msg.pose.pose.position
                initialized[topic] = True
                distance_traveled[topic].append(0)
                continue
            
            current_position = msg.pose.pose.position  
            dx = np.square(current_position.x - previous_position[topic].x)
            dy = np.square(current_position.y - previous_position[topic].y) 
            dz = np.square(current_position.z - previous_position[topic].z)
            distance_traveled[topic].append(distance_traveled[topic][-1] + np.sqrt(dx + dy + dz))
            previous_position[topic] = current_position     

    # Match timestamps
    matched_distances = {}
    eps = 3e-2
    adjust_count = 0
    for method in methods:
        indices = np.searchsorted(distance_time['ground_truth'], timestamps[method])
        for i in range(len(indices)):
            d1 = abs(distance_time['ground_truth'][indices[i]] - timestamps[method][i])
            if d1 > eps:
                if indices[i] > 0:
                    d2 = abs(distance_time['ground_truth'][indices[i]-1] - timestamps[method][i])
                    if d2 < d1:
                        indices[i] = indices[i] - 1
                adjust_count += 1
        matched_distances[method] = np.take(distance_traveled['ground_truth'],indices)

    print("Number to adjust = {}".format(adjust_count))
    # Compute metrics
    error_percentage_trans = {}
    error_percentage_rot = {}
    err_perc_trans_average = {}
    err_perc_rot_average = {}
    i_start = {}
    # Start when we have moved at least min_move m
    i_start['ground_truth'] = np.searchsorted(distance_traveled['ground_truth'], min_move)
    t_start = distance_time['ground_truth'][i_start['ground_truth']]

    for method in methods:
        error_percentage_trans[method] = ape_trans[method]/matched_distances[method]
        error_percentage_rot[method] = ape_rot[method]/matched_distances[method]
        i_start[method] = np.searchsorted(timestamps[method], t_start)
        err_perc_trans_average[method] = np.mean(error_percentage_trans[method][i_start[method]:])
        err_perc_rot_average[method] = np.mean(error_percentage_rot[method][i_start[method]:])
        print("Translational Error Percentage for method {} is {} %".format(method, err_perc_trans_average[method]))
        print("Rotational error deg/m for method {} is {} %".format(method, err_perc_rot_average[method]))


    # Plot distance vs time
    fig = plt.figure() 
    for method in methods:
        plt.plot(timestamps[method], matched_distances[method], label=method, linewidth=3)
    
    title = "DistancevsTime"
    plt.xlabel("Time (s)")
    plt.ylabel("Distance Traveled (m)")
    plt.legend(prop={'size': 30})
    if fullscreen:
        enable_full_screen()
    plt.tight_layout()
    fig.savefig(results_dir + "/plots/" + title)
    if show:
        plt.show()
    plt.close()

    
    # Plot APE vs distance
    fig = plt.figure() 
    for method in methods:
        plt.plot(matched_distances[method], ape_trans[method], label=method, linewidth=3)
    
    title = "APETransvsDistance"
    plt.xlabel("Distance Traveled (m)")
    plt.ylabel("APE (trans) (m)")
    plt.legend(prop={'size': 30})
    if fullscreen:
        enable_full_screen()
    plt.tight_layout()
    fig.savefig(results_dir + "/plots/" + title)
    if show:
        plt.show()
    plt.close() 

    # Plot APE Rot vs distance
    fig = plt.figure() 
    for method in methods:
        plt.plot(matched_distances[method], ape_rot[method], label=method, linewidth=3)
    
    title = "APERotvsDistance"
    plt.xlabel("Distance Traveled (m)")
    plt.ylabel("APE (rot) (deg)")
    plt.legend(prop={'size': 30})
    if fullscreen:
        enable_full_screen()
    plt.tight_layout()
    fig.savefig(results_dir + "/plots/" + title)
    if show:
        plt.show()
    plt.close() 
    
    # Plot Error % vs distance
    fig = plt.figure() 
    for method in methods:
        plt.plot(matched_distances[method][i_start[method]:], error_percentage_trans[method][i_start[method]:], label=method, linewidth=3)
    
    title = "ErrorPercvsdistance"
    plt.xlabel("Distance Traveled (m)")
    plt.ylabel("Translational Error (%)")
    plt.legend(prop={'size': 30})
    # plt.yscale("log")
    if fullscreen:
        enable_full_screen()
    plt.tight_layout()
    fig.savefig(results_dir + "/plots/" + title)
    if show:
        plt.show()
    plt.close() 

    # Plot Rotational Error deg/m vs distance
    fig = plt.figure() 
    for method in methods:
        plt.plot(matched_distances[method][i_start[method]:], error_percentage_rot[method][i_start[method]:], label=method, linewidth=3)
    
    title = "ErrorPercRotvsdistance"
    plt.xlabel("Distance Traveled (m)")
    plt.ylabel("Rotational Error (deg/m)")
    plt.legend(prop={'size': 30})
    # plt.yscale("log")
    if fullscreen:
        enable_full_screen()
    plt.tight_layout()
    fig.savefig(results_dir + "/plots/" + title)
    if show:
        plt.show()
    plt.close() 
    

    err_perc_average = {}
    err_perc_average['trans'] = err_perc_trans_average
    err_perc_average['rot'] = err_perc_rot_average

    return err_perc_average
    

def write_main_results_to_csv(methods, results_dir, err_perc, distributions, bandwidth_stats):
    import csv 
    # Distributions has [cpus, mems, rates, delays, times]

    ape_stats = {}
    trans_ape_stats = {}
    rot_ape_stats = {}

    # Read in the APE results
    with open(results_dir + "combined_ape.csv") as apefile:
        reader = csv.reader(apefile, delimiter=',', quotechar="|")
        firstline = True
        for row in reader:
            if firstline:
                firstline = False
            else:
                name = row[0][12:-4]
                ape_stats[name] = []
                for el in row[1:]:
                    ape_stats[name].append(float(el))
    with open(results_dir + "trans_ape.csv") as apefile:
        reader = csv.reader(apefile, delimiter=',', quotechar="|")
        firstline = True
        for row in reader:
            if firstline:
                firstline = False
            else:
                name = row[0][12:-10]
                trans_ape_stats[name] = []
                for el in row[1:]:
                    trans_ape_stats[name].append(float(el))
    with open(results_dir + "rot_ape.csv") as apefile:
        reader = csv.reader(apefile, delimiter=',', quotechar="|")
        firstline = True
        for row in reader:
            if firstline:
                firstline = False
            else:
                name = row[0][12:-8]
                rot_ape_stats[name] = []
                for el in row[1:]:
                    rot_ape_stats[name].append(float(el))

    # Write out the summary CSV
    with open(results_dir + "main_results_tmp.csv", 'w') as outfile:
        writer = csv.writer(outfile, delimiter=",", quotechar='|', quoting=csv.QUOTE_MINIMAL)
        writer.writerow([   'Method',
                            'Max Translational APE (m)', 
                            'Mean Translational Error %',
                            'Max Rotational APE (deg)',  
                            'Mean Rotational Error (deg/m)',  
                            'Average CPU (%)', 
                            'Max CPU (%)', 
                            'Max memory (GB)',
                            'Max Bandwidth (KB/s)',
                            'Mean Bandwidth (KB/s)',
                            'Max odom delay (s)',
                            'Mean odom delay (s)',
                             ])

        # Add more fields later: Rotation, 'Avg Memory Rate (GB)'
        i = 0
        bw_stat_1 = 0.0
        bw_stat_2 = 0.0
        if bandwidth_stats:
            bw_stat_1 = bandwidth_stats["max"]
            bw_stat_2 = bandwidth_stats["mean"]
            
        for method in methods:
            writer.writerow([   
                            method,
                            trans_ape_stats[method][0], 
                            err_perc['trans'][method], 
                            rot_ape_stats[method][0], 
                            err_perc['rot'][method], 
                            distributions[0][i][2],
                            distributions[0][i][1],
                            distributions[1][i][1],
                            bw_stat_1,
                            bw_stat_2,
                            distributions[3][i][1],
                            distributions[3][i][2]
                            ])
            i = i+1

    # Transpose the csv
    #from itertools import zip
    a = zip(*csv.reader(open(results_dir + "main_results_tmp.csv", "rt")))
    csv.writer(open(results_dir + "main_results.csv", "wt")).writerows(a)
    os.remove(results_dir + "main_results_tmp.csv")

def analyze_bandwidth(lamp_data_path, result_data_path):
    input_file_name = lamp_data_path + "/bandwidth.txt"
    output_file_name = result_data_path + "/bandwidth_result.txt"
    # read from saved text file 
    input_file = open(input_file_name, 'r') 
    lines = input_file.readlines() 

    max_bw = 0
    sum_of_mean_bw = 0 
    count_bw = 0
    first_message = ""
    last_message = ""

    for line in lines: 
        line_list = line.split()
        if line_list[0] == "mean:":
            try:
                if count_bw == 0:
                    first_message = line
                last_message = line
                maxstr = line_list[5]
                meanstr = line_list[1]
                if (float(maxstr[:-2]) > max_bw):
                    max_bw = float(maxstr[:-2])
                sum_of_mean_bw += float(meanstr[:-2])
                count_bw += 1
            except:
                continue

    input_file.close()

    # write output
    if count_bw == 0:
        bandwidth_stats = {}
        bandwidth_stats["max"] = 0.0
        bandwidth_stats["mean"] = 0.0
        return bandwidth_stats

    ouput_content = ["max: " + str(max_bw) + "\n", "mean: " + str(sum_of_mean_bw/count_bw) + "\n"]
    ouput_content.append("first message:" + first_message)
    ouput_content.append("last message:" + last_message)
    output_file = open(output_file_name, 'w')
    output_file.writelines(ouput_content)
    output_file.close()
    
    bandwidth_stats = {}
    bandwidth_stats["max"] = max_bw
    bandwidth_stats["mean"] = sum_of_mean_bw/count_bw
    return bandwidth_stats

def evaluate_loop_closures(pose_graph_bag, pose_graph_topic, 
    loop_closure_bag, loop_closure_topic, gt_odom_bag, gt_odom_topic, output_dir):
    loop_closure_eval = LoopClosureEvaluator(
            pose_graph_bag, pose_graph_topic, loop_closure_bag, loop_closure_topic, gt_odom_bag, gt_odom_topic)
    loop_closure_eval.evaluate()
    if loop_closure_eval.get_num_loopclosures() > 0:
        loop_closure_eval.save_accuracy_plot(filename=output_dir + "/lc_accuracy.png")
        loop_closure_eval.save_inlier_accuracy_plot(filename=output_dir + "/inlier_lc_accuracy.png")
        if loop_closure_eval.get_num_inliers() > 0:
            loop_closure_eval.save_avg_accuracy_plot(filename=output_dir + "/per_node_lc_accuracy.png")
            loop_closure_eval.save_avg_inlier_accuracy_plot(filename=output_dir + "/per_node_inlier_lc_accuracy.png")
        loop_closure_eval.save_covariance_plot(filename=output_dir + "/lc_covariance.png")

def get_loop_closures_stats(pose_graph_bag, pose_graph_topic, 
    loop_closure_bag, loop_closure_topic, output_dir):
    loop_closure_stats = LoopClosureEvaluator(
            pose_graph_bag, pose_graph_topic, loop_closure_bag, loop_closure_topic)
    loop_closure_stats.save_stats(output_dir)

def imageToPDF(filename):
    from PIL import Image
    img = Image.open(filename)
    img1 = img.convert('RGB')
    img1.save(filename[:-3] + "pdf")

def creatOutputPDF(results_dir):

    # Convert images desired into pdf 
    ape_rot = results_dir + "/plots/APERotvsDistance.png"
    imageToPDF(ape_rot)
    ape_trans = results_dir + "/plots/APETransvsDistance.png"
    imageToPDF(ape_trans)
    cpu = results_dir + "/plots/Raw_Cpu_Load.png"
    imageToPDF(cpu)
    mem = results_dir + "/plots/Raw_Memory_Load.png"
    imageToPDF(mem)
    delay = results_dir + "/plots/Raw_Odometry_Delay.png"
    imageToPDF(delay)

    # Create pdf and start adding pages
    from PyPDF2 import PdfFileReader, PdfFileWriter
    pdf_writer = PdfFileWriter()

    # Add trajectory
    input_pdf = PdfFileReader(results_dir + "/plots/traj_plots.pdf")
    pdf_writer.addPage(input_pdf.getPage(0))

    # Add colored error plots
    input_pdf = PdfFileReader(results_dir + "/plots/lamp_ape_plots.pdf")
    pdf_writer.addPage(input_pdf.getPage(1))
    
    # Add error vs distance
    input_pdf = PdfFileReader(results_dir + "/plots/APETransvsDistance.pdf")
    pdf_writer.addPage(input_pdf.getPage(0))
    input_pdf = PdfFileReader(results_dir + "/plots/APERotvsDistance.pdf")
    pdf_writer.addPage(input_pdf.getPage(0))

    # Add box plot
    input_pdf = PdfFileReader(results_dir + "/plots/combined_ape_plots.pdf")
    pdf_writer.addPage(input_pdf.getPage(3))

    # More Stats on overall error
    input_pdf = PdfFileReader(results_dir + "/plots/combined_ape_plots.pdf")
    pdf_writer.addPage(input_pdf.getPage(1))
    pdf_writer.addPage(input_pdf.getPage(4))

    # X, Y, Z error
    input_pdf = PdfFileReader(results_dir + "/plots/traj_plots.pdf")
    pdf_writer.addPage(input_pdf.getPage(1))

    # Computation plots
    input_pdf = PdfFileReader(results_dir + "/plots/Raw_Cpu_Load.pdf")
    pdf_writer.addPage(input_pdf.getPage(0))
    input_pdf = PdfFileReader(results_dir + "/plots/Raw_Memory_Load.pdf")
    pdf_writer.addPage(input_pdf.getPage(0))
    input_pdf = PdfFileReader(results_dir + "/plots/Raw_Odometry_Delay.pdf")
    pdf_writer.addPage(input_pdf.getPage(0))

    # Write to file 
    with open(results_dir + "/main_results.pdf", 'wb') as outfile:
        pdf_writer.write(outfile)
