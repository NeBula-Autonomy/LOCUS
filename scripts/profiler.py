"""
Description: 
    - A script to profile Rate/Delay performance of a logged ROS topic
Author: 
    - Matteo Palieri, NASA Jet Propulsion Laboratory
"""
from matplotlib import pyplot as plt 

plt.style.use('dark_background')

def plot_and_save_data(title, data):  
    fig = plt.figure()  
    plt.plot(data, linewidth=5)
    plt.title(title)
    plt.show()
    fig.savefig(title)

def get_avg(data): 
    return reduce(lambda x,y: x+y, data)/len(data) 

def save_report(hz_data, delay_data):
    plot_and_save_data("Rate", hz_data)
    plot_and_save_data("Delay", delay_data)
    avg_hz_str = str(get_avg(hz_data)) 
    avg_delay_str = str(get_avg(delay_data)) 
    print("Rate avg: " + avg_hz_str)
    print("Delay avg: " + avg_delay_str)
    outfile = open("report.txt", "w")
    outfile.write("Rate avg: " + avg_hz_str + "\n" + "Rate delay: " + avg_delay_str + "\n")
    outfile.close()

def main(): 
    hz = open("hz.txt")
    delay = open("delay.txt")    
    hz_data, delay_data = [], []
    for el in hz: 
        if "average rate:" in el: 
            splitted = el.split(": ")
            hz_data.append(float(splitted[1]))
    for el in delay: 
        if "average delay:" in el: 
            splitted = el.split(": ")
            delay_data.append(float(splitted[1]))
    save_report(hz_data, delay_data)

if __name__=="__main__":
    main()