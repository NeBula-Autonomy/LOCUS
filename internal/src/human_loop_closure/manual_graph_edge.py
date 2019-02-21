#!/usr/bin/env python
import rospy, sys
from laser_loop_closure.srv import ManualLoopClosure

def connect(key_from, key_to):
    rospy.init_node('human_operator_loop_closure')
    add_factor = rospy.ServiceProxy('/blam/blam_slam/add_factor', ManualLoopClosure)
    if add_factor(key_from, key_to).success:
        print('Successfully added a factor between %i and %i to the graph.' % (key_from, key_to))
    else:
        print('An error occurred while trying to add a factor between %i and %i.' % (key_from, key_to))

if __name__ == '__main__':
    try:
        if len(sys.argv) < 3:
            print('Usage: %s from_key to_key' % sys.argv[0])
        else:
            connect(int(sys.argv[1]), int(sys.argv[2]))
    except rospy.ROSInterruptException: pass
