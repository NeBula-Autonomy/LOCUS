#!/usr/bin/env python
import rospy, sys
from blam_slam.srv import RemoveFactor
from add_factor import yes_or_no

def connect(key_from, key_to):
    rospy.init_node('remove_factor_client')
    remove_factor = rospy.ServiceProxy('/blam/blam_slam/remove_factor', RemoveFactor)
    response = remove_factor(key_from, key_to, False)
    if response.confirm:
        if response.success:
            if yes_or_no('The factor to be removed from the factor graph is now visualized in RViz.\nDo you confirm the removal?'):
                response = remove_factor(key_from, key_to, True)
                if response.success:
                    print('Successfully removed a factor between %i and %i from the graph.' % (key_from, key_to))
                else:
                    print('An error occurred while trying to remove a factor between %i and %i.' % (key_from, key_to))
            else:
                print('Aborted manual loop closure.')
                remove_factor(0, 0, False)  # remove edge visualization
        else:
            print('Error: One or more of the keys %i and %i do not exist, or they are adjacent.' % (key_from, key_to))
            remove_factor(0, 0, False)  # remove edge visualization

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
