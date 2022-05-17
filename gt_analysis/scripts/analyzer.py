"""
Description:  
    - Operates a full, and fully automatic analyis of user requested methods    
      providing statistics on a broad range of metrics such as 
        - Cpu Load 
        - Memory Usage  
        - Odometry Rate 
        - Odometry Delay
        - Lidar Processing Time 
        - Absolute Pose Error (APE)
        - Relative Pose Error (RPE)
        - APE over multiple distances 
        - Volume/Size covered during exploration
Author: 
    - Matteo Palieri, NASA Jet Propulsion Laboratory
    - Andrzej Reinke, NASA Jet Propulsion Laboratory 
"""

import sys
import rospkg
import shutil


sys.path.insert(1, rospkg.RosPack().get_path('gt_analysis') + "/utilities")
from utilities import *

def main():

    global methods 
        
    if len(sys.argv)<5:
        print("Example Usage: python analyzer.py data_dir robot_name ground_truth_topic run_number")
        sys.exit(1)  

    data_dir = sys.argv[1]
    robot_name = sys.argv[2]
    ground_truth_topic = sys.argv[3]
    run_number = sys.argv[4]
    params = {}

    settings_filename = rospkg.RosPack().get_path('gt_analysis') + "/config/analyzer_settings.yaml"

    print("\n#################")
    print("Starting Analyzer")
    print("#################\n")

    with open(settings_filename) as file:
        params = yaml.load(file, Loader=yaml.FullLoader)
        for k in params:
            print(k + " : " + str(params[k])) 
    print("\n")
    
    base_station_name = params["base_station_name"]
    b_clear_results_dir = params["b_clear_results_dir"]
    b_enable_dark_mode = params["b_enable_dark_mode"] 
    b_visualize_raw_data = params["b_visualize_raw_data"]
    b_save_raw_data_plots = params["b_save_raw_data_plots"]
    b_visualize_normal_distributions = params["b_visualize_normal_distributions"]
    b_save_raw_data_distribution_plots = params["b_save_raw_data_distribution_plots"]
    b_fullscreen = params["b_fullscreen"]
    b_plot_volume_vs_size = params["b_plot_volume_vs_size"]
    b_save_volume_vs_size_plot = params["b_save_volume_vs_size_plot"]

    b_save_traj_plots = ["b_save_traj_plots"]

    # APE
    b_compute_ape = params["b_compute_ape"]
    b_align_origin = params["b_align_origin"]
    b_plot_combined_ape = params["b_plot_combined_ape"]
    b_show_ape_plots = params["b_show_ape_plots"]
    b_save_individual_ape_plots = params["b_save_individual_ape_plots"]
    b_save_combined_ape_plots = params["b_save_combined_ape_plots"]
    b_compute_rpe = params["b_compute_rpe"]
    b_plot_rpe = params["b_plot_rpe"]
    checkpoints  = params["checkpoints"]
    b_compute_ape_over_distances = params["b_compute_ape_over_distances"]   
    b_plot_ape_over_distances = params["b_plot_ape_over_distances"]
    b_save_results = params["b_save_results"]

    # Distance metrics
    b_compute_distance_based_metrics = params["b_compute_distance_based_metrics"]
    b_show_distance_plots = params["b_show_distance_plots"]

    if params["all_visualization_off"]:
        # Stops any plots from visalizating (plots can still save)
        b_visualize_raw_data = False
        b_visualize_normal_distributions = False
        b_plot_volume_vs_size = False
        b_plot_ape_over_distances = False
       
    results_dir = data_dir + "/results_" + run_number + "/"
    if not os.path.exists(results_dir):
        os.mkdir(results_dir)
    else:
        # Clean up
        if b_clear_results_dir:
            if os.path.exists(results_dir + "combined_ape.csv"):
                os.remove(results_dir + "combined_ape.csv")
            if os.path.exists(results_dir + "trans_ape.csv"):
                os.remove(results_dir + "trans_ape.csv")
            if os.path.exists(results_dir + "rot_ape.csv"):
                os.remove(results_dir + "rot_ape.csv")
            if os.path.exists(results_dir + "main_results.csv"):
                os.remove(results_dir + "main_results.csv")

    os.chdir(results_dir)

    # Make and clean plot directory
    plot_dir = data_dir + "/results_" + run_number + "/plots"
    if not os.path.exists(plot_dir):
        os.mkdir(plot_dir)
    else:
        shutil.rmtree(plot_dir)
        os.mkdir(plot_dir)

    default_case = not params["specify_non_default_methods"]

    if not default_case:
        for i in range(len(params["specific_methods"])):
            methods.append(params["specific_methods"][i])
    else:
        # methods.append("lamp")
        methods.append("locus")

    all_cpus, all_mems, all_rates, all_delays, all_times = [], [], [], [], []

    for method in methods: 
        all_cpus.append([])
        all_mems.append([])
        all_rates.append([])
        all_delays.append([])
        all_times.append([])
    
    counter = 0 

    for method in methods: 

        print("Profiling " + method + " ...\n")

        prefix = data_dir + "/" + method + "_" + run_number

        files = ["cpu.bag",  "mem.bag", "time.bag", "rate.txt", "delay.txt"]
        available_files = []

        for el in files:     
            if el in os.listdir(prefix):
                available_files.append(el)
            else:
                print("\n----------------------------------------")
                print("WARNING " + el + " not found for " + method)
                print("----------------------------------------\n")

        cpu, mem, rate, delay, times = None, None, None, None, None 

        for el in available_files: 
            
            if el.split(".")[0] == "cpu": 
                cpu = rosbag.Bag(prefix + "/" + el)
                for topic, msg, t in cpu.read_messages():
                    all_cpus[counter].append(msg.data)  
            
            if el.split(".")[0] == "mem": 
                mem = rosbag.Bag(prefix + "/" + el)
                for topic, msg, t in mem.read_messages():
                    all_mems[counter].append(msg.data/1e9)  
            
            if el.split(".")[0] == "rate": 
                rate = open(prefix + "/" + el)
                for r in rate:
                    if "average rate:" in r: 
                        splitted = r.split(": ")
                        if np.size(splitted) > 1:
                            all_rates[counter].append(float(splitted[1])) 

            if el.split(".")[0] == "delay": 
                delay = open(prefix + "/" + el)
                for d in delay: 
                    if "average delay:" in d:
                        splitted = d.split(": ")
                        if np.size(splitted) > 1:
                            all_delays[counter].append(float(splitted[1]))

            if el.split(".")[0] == "time": 
                time = rosbag.Bag(prefix + "/" + el)
                for topic, msg, t in time.read_messages():
                    all_times[counter].append(msg.data)   

        counter = counter + 1 

    min_sizes = []  

    data = [all_cpus, all_mems, all_rates, all_delays, all_times]

    for entry in data: 
        if (max(len(el) for el in entry) != 0):
            if (min(len(el) for el in entry if len(el) !=0) != 0):
                min_sizes.append(min(len(el) for el in entry if len(el) !=0))
        else: 
            min_sizes.append(0) 

    all_cpus = [el[:min_sizes[0]] for el in all_cpus]
    all_mems = [el[:min_sizes[1]] for el in all_mems]
    all_rates = [el[:min_sizes[2]] for el in all_rates]
    all_delays = [el[:min_sizes[3]] for el in all_delays]
    all_times = [el[:min_sizes[4]] for el in all_times]
    
    distributions = evaluate_normal_distribution(data)      
    cpu_results, mem_results, rate_results, delay_results, time_results = [], [], [], [], []

    for d in range(len(distributions)): 

        distribution = distributions[d]

        for i in range(len(methods)): 

            min_val = distribution[i][0]
            max_val = distribution[i][1]
            avg_val = distribution[i][2]
            var_val = distribution[i][3]
            current_data = [min_val, max_val, avg_val, var_val]
            sigma = math.sqrt(var_val)
            x = np.linspace(avg_val - 3*sigma, avg_val + 3*sigma, 100) 

            if d==0: 
                cpu_results.append([x, stats.norm.pdf(x, avg_val, sigma), methods[i]])
                if b_save_results:
                    current_title = results_dir + "cpu_results_" + methods[i] + ".txt"                     
                    save_txt(current_title, current_data)
            if d==1: 
                mem_results.append([x, stats.norm.pdf(x, avg_val, sigma), methods[i]])
                if b_save_results:
                    current_title = results_dir + "mem_results_" + methods[i] + ".txt"                     
                    save_txt(current_title, current_data)
            if d==2: 
                rate_results.append([x, stats.norm.pdf(x, avg_val, sigma), methods[i]])
                if b_save_results:
                    current_title = results_dir + "rate_results_" + methods[i] + ".txt"                     
                    save_txt(current_title, current_data)
            if d==3: 
                delay_results.append([x, stats.norm.pdf(x, avg_val, sigma), methods[i]])
                if b_save_results:
                    current_title = results_dir + "delay_results_" + methods[i] + ".txt"                     
                    save_txt(current_title, current_data)
            if d==4: 
                time_results.append([x, stats.norm.pdf(x, avg_val, sigma), methods[i]])
                if b_save_results:
                    current_title = results_dir + "time_results_" + methods[i] + ".txt"                     
                    save_txt(current_title, current_data)

    if b_enable_dark_mode: 
        enable_dark_mode()

    if b_visualize_raw_data or b_save_raw_data_plots:
        visualize_raw_data("Raw_Cpu_Load", "CPU Load (% CPU)", all_cpus, results_dir, b_visualize_raw_data, b_fullscreen)
        visualize_raw_data("Raw_Memory_Load", "Virtual Memory (GB)", all_mems, results_dir, b_visualize_raw_data, b_fullscreen)
        visualize_raw_data("Raw_Odometry_Rate", "Rate (Hz)", all_rates, results_dir, b_visualize_raw_data, b_fullscreen)
        visualize_raw_data("Raw_Odometry_Delay", "Delay (s)", all_delays, results_dir, b_visualize_raw_data, b_fullscreen)
        visualize_raw_data("Raw_Lidar_Processing_Time", "Processing time (s)", all_times, results_dir, b_visualize_raw_data, b_fullscreen)

    if b_visualize_normal_distributions or b_save_raw_data_distribution_plots:
        all_results = {}
        all_results["cpu"] = [cpu_results, "Cpu_Load"]
        all_results["mem"] = [mem_results, "Memory_Load"]        
        all_results["rate"] = [rate_results, "Odometry_Rate"]
        all_results["delay"] = [delay_results, "Odometry_Delay"]
        all_results["time"] = [time_results, "Lidar_Processing_Time"]
        visualize_and_save_normal_distributions(all_results, results_dir, b_visualize_normal_distributions, b_fullscreen)
    
    bandwidth_stats = {}
    if "lamp" in methods:         
        if b_plot_volume_vs_size or b_save_volume_vs_size_plot:
            volume, size, timestamps = [], [], []
            map_info = rosbag.Bag( data_dir + "/" + "lamp" + "_" + run_number + "/map_info.bag")
            for topic, msg, t in map_info.read_messages():
                size.append(msg.size)  
                volume.append(msg.volume)
                timestamps.append(t.to_sec())
            timestamps = [el-timestamps[0] for el in timestamps]
            plot_volume_size(volume, size, timestamps, results_dir, b_plot_volume_vs_size, b_fullscreen)        
        pose_graph_to_odometry(data_dir + "/" + "lamp" + "_" + run_number + "/", robot_name)
        # track keyed scans bandwidth 
        bandwidth_stats = analyze_bandwidth(data_dir + "/" + "lamp" + "_" + run_number, results_dir)

        # analyze loop closures 
        pose_graph_rosbag = data_dir + "/" + "lamp" + "_" + run_number + "/pose_graph.bag"
        pose_graph_topic = "/" + robot_name + "/lamp/pose_graph"
        loop_closure_rosbag = data_dir + "/" + "lamp" + "_" + run_number + "/loop_closures.bag"
        loop_closure_topic = "/" + robot_name + "/lamp/laser_loop_closures"
        gt_odom_rosbag = data_dir + "/ground_truth/odometry.bag"
        gt_odom_topic = ground_truth_topic
        # evaluate_loop_closures(
        #     pose_graph_rosbag, pose_graph_topic, loop_closure_rosbag, loop_closure_topic, gt_odom_rosbag, gt_odom_topic, results_dir)

    all_methods_string = " "
    for method in methods: 
        all_methods_string = all_methods_string + method + " "

    bash_command = "python $(rospack find gt_analysis)/utilities/aggregate_odometries.py " + data_dir + " " + robot_name + " " + ground_truth_topic + " " + run_number
    os.system(bash_command)

    #-----------------------------------------------------------------------------------------------------------
    ### Plot trajectories ###
    #-----------------------------------------------------------------------------------------------------------
    
        
    bash_command = "evo_config set plot_fontfamily arial plot_fontscale 1.5 plot_seaborn_style whitegrid plot_figsize 14 10"
    os.system(bash_command)
    evo_options_string = make_evo_options_string("traj", "", b_align_origin, b_show_ape_plots, b_save_traj_plots)
    bash_command = "evo_traj bag  aggregated_odometries.bag ground_truth" + all_methods_string + "--ref ground_truth " + evo_options_string
    os.system(bash_command)

    #-----------------------------------------------------------------------------------------------------------
    ### APE Processing with EVO ###
    #-----------------------------------------------------------------------------------------------------------
    if b_compute_ape:
        # Create directory
        ape_dir = data_dir + "/results_" + run_number + "/ape_results"
        if not os.path.exists(ape_dir):
            os.mkdir(ape_dir)
        else:
            shutil.rmtree(ape_dir)
            os.mkdir(ape_dir)


        # Get APE for each method
        for method in methods:
            # Make options string from params
            evo_options_string = make_evo_options_string("single", method, b_align_origin, b_show_ape_plots, b_save_individual_ape_plots)

            # Run bash command to compute APE for each method
            bash_command = "evo_ape bag  aggregated_odometries.bag ground_truth " + method + " " + evo_options_string + " --save_results " + ape_dir + "/" + method + ".zip"
            os.system(bash_command)
        if b_plot_combined_ape:
            # Make options string from params
            evo_options_string = make_evo_options_string("combined", "", False, b_show_ape_plots, b_save_combined_ape_plots)

            # Run bash command to show combined results
            bash_command = "evo_res ape_results/*.zip --use_filenames" + " " + evo_options_string
            os.system(bash_command)
        for method in methods:
            # Run for translation only
            evo_options_string = make_evo_options_string("trans", method, b_align_origin, b_show_ape_plots, b_save_individual_ape_plots)
            bash_command = "evo_ape bag  aggregated_odometries.bag ground_truth " + method + " --pose_relation trans_part" + evo_options_string + " --save_results " + ape_dir + "/" + method + "_trans.zip"
            os.system(bash_command)
            # Run for rotation only
            evo_options_string = make_evo_options_string("rot", method, b_align_origin, b_show_ape_plots, b_save_individual_ape_plots)
            bash_command = "evo_ape bag  aggregated_odometries.bag ground_truth " + method + " --pose_relation angle_deg" + evo_options_string + " --save_results " + ape_dir + "/" + method + "_rot.zip"
            os.system(bash_command)
        # Run bash command to show combined results
        evo_options_string = make_evo_options_string("combined_trans", "", False, False, b_save_combined_ape_plots)
        bash_command = "evo_res ape_results/*trans.zip --use_filenames" + " " + evo_options_string
        os.system(bash_command)
        evo_options_string = make_evo_options_string("combined_rot", "", False, False, b_save_combined_ape_plots)
        bash_command = "evo_res ape_results/*rot.zip --use_filenames" + " " + evo_options_string
        os.system(bash_command)

    #-----------------------------------------------------------------------------------------------------------
    ### Distance based error metrics ###
    #-----------------------------------------------------------------------------------------------------------
    if b_compute_distance_based_metrics:
        # Extract zip contents
        dest = results_dir + "ape_results"
        for el in os.listdir(dest): 
            if os.path.isfile(dest + "/" + el) and el.split(".")[1] == "zip": 
                with zipfile.ZipFile(dest + "/" + el, 'r') as zip_ref:
                    zip_ref.extractall(dest + "/" + el.split(".")[0])
        
        # Compute the distance metrics
        err_perc = compute_distance_based_error_metrics(methods, results_dir, b_show_distance_plots, b_fullscreen)


    #-----------------------------------------------------------------------------------------------------------
    ### APE At checkpoints with EVO ###
    #-----------------------------------------------------------------------------------------------------------
    if b_compute_ape_over_distances: 
        segment_aggregated_odometries(methods, checkpoints)
        for checkpoint in checkpoints:
            bash_command = "bash $(rospack find gt_analysis)/utilities/ape_from_aggregated_odometries_parser.bash " + \
                           "aggregated_odometries_" + str(checkpoint) + ".bag" + " " + \
                           "ape_results_" + str(checkpoint) + " " + all_methods_string 
            os.system(bash_command)
            dest = os.getcwd() + "/ape_results_" + str(checkpoint)
            for el in os.listdir(dest): 
                if os.path.isfile(dest + "/" + el) and el.split(".")[1] == "zip": 
                    with zipfile.ZipFile(dest + "/" + el, 'r') as zip_ref:
                        zip_ref.extractall(dest + "/" + el.split(".")[0])
        if b_plot_ape_over_distances:
            boxplot_from_npz(methods, checkpoints, results_dir, b_show_ape_plots, b_fullscreen)

    #-----------------------------------------------------------------------------------------------------------
    ### RPE Processing with EVO ###
    #-----------------------------------------------------------------------------------------------------------
    if b_compute_rpe: 
        # Create directory
        rpe_dir = data_dir + "/results_" + run_number + "/rpe_results"
        if not os.path.exists(rpe_dir):
            os.mkdir(rpe_dir)
        else:
            shutil.rmtree(rpe_dir)
            os.mkdir(rpe_dir)

        # Get RPE for each method
        for method in methods:
            # Make options string from params
            # evo_options_string = make_evo_options_string("single", method, b_align_origin, b_show_ape_plots, b_save_individual_ape_plots)

            # Run bash command to compute APE for each method
            bash_command = "evo_rpe bag  aggregated_odometries.bag ground_truth " + method + " --delta 1 --delta_unit m --save_results " + rpe_dir + "/" + method + ".zip"
            os.system(bash_command)
        if b_plot_rpe:
            # Make options string from params
            # evo_options_string = make_evo_options_string("combined", "", b_align_origin, b_show_rpe_plots, b_plot_rpe)

            # Run bash command to show combined results
            bash_command = "evo_res rpe_results/*.zip --merge --use_filenames" #+ " " + evo_options_string
            os.system(bash_command)


        # bash_command = "bash $(rospack find gt_analysis)/utilities/rpe_from_aggregated_odometries.bash " + all_methods_string 
        # os.system(bash_command)
        # if b_plot_rpe:
        #     bash_command = "bash $(rospack find gt_analysis)/utilities/plot_rpe_results.bash " + all_methods_string 
        #     os.system(bash_command)

    #-----------------------------------------------------------------------------------------------------------
    ### Export main stats to CSV ###
    #-----------------------------------------------------------------------------------------------------------
    if b_save_results:
        write_main_results_to_csv(methods, results_dir, err_perc, distributions, bandwidth_stats)
        # creatOutputPDF(results_dir)


if __name__=="__main__":
    main()
