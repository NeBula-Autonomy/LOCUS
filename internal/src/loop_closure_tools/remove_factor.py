#!/usr/bin/env python
import rospy, sys
from blam_slam.srv import RemoveFactor

def connect(key_from, key_to):
    rospy.init_node('remove_factor_client')
    remove_factor = rospy.ServiceProxy('/blam/blam_slam/remove_factor', RemoveFactor)
    if remove_factor(key_from, key_to).success:
        print('Successfully removed a factor between %i and %i from the graph.' % (key_from, key_to))
    else:
        print('An error occurred while trying to remove a factor between %i and %i.' % (key_from, key_to))

if __name__ == '__main__':
    try:
        if len(sys.argv) < 3 or "h" in sys.argv[1].lower():
            print("""Usage:
    python {arg0} <from_key> <to_key>
        Removes an edge between keys from_key and to_key from the pose graph.
            """.format(arg0=sys.argv[0]))
        else:
            connect(int(sys.argv[1]), int(sys.argv[2]))

    except rospy.ROSInterruptException: pass
