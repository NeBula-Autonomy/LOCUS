#!/usr/bin/env python
import rospy, sys
from blam_slam.srv import Restart

def connect(namespace):
    rospy.init_node('restart_client')
    restart = rospy.ServiceProxy(namespace + '/blam_slam/restart', Restart)
    if restart('posegraph_backup.zip').success:
        print('Successfully restarted from last saved graph')
    else:
        print('Error: posegraph_backup.zip missing from directory or has been corrupted')

if __name__ == '__main__':
    try:
        if len(sys.argv) > 1:
            print('Usage:python %s ' % sys.argv[0])
        if len(sys.argv) < 2:
            connect('husky')
        else:
            connect(sys.argv[1])
    except rospy.ROSInterruptException: pass
