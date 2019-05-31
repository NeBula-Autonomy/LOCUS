#!/usr/bin/env python
import rospy, sys
from blam_slam.srv import LoadGraph

def connect(filename, namespace):
    rospy.init_node('load_graph_client')
    load_graph = rospy.ServiceProxy(namespace + '/blam_slam/load_graph', LoadGraph)
    if load_graph(filename).success:
        print('Successfully loaded from last saved graph')
    else:
        print('Error: posegraph_backup.zip missing from directory or has been corrupted')

if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
            print('Usage: %s filename.zip' % sys.argv[0])
        if len(sys.argv) == 2:
            connect(sys.argv[1], 'husky')
        else:
            connect(sys.argv[1], sys.argv[2])
    except rospy.ROSInterruptException: pass

