#!/usr/bin/env python
import rospy, sys
from blam_slam.srv import SaveGraph

def connect(filename, namespace):
    rospy.init_node('save_graph_client')
    save_graph = rospy.ServiceProxy(namespace + '/blam_slam/save_graph', SaveGraph)
    if save_graph(filename).success:
        print('Successfully saved the pose graph to %s.' % filename)
    else:
        print('An error occurred while trying to save the pose graph to %s.' % filename)

if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
            print('Usage: %s filename.zip' % sys.argv[0])
        if len(sys.argv) == 2:
            connect(sys.argv[1], 'husky')
        else:
            connect(sys.argv[1], sys.argv[2])
    except rospy.ROSInterruptException: pass
