#!/usr/bin/env python
import rospy, sys
from blam_slam.srv import AddFactor

def connect(key_from, key_to, quat):
    rospy.init_node('add_factor_client')
    add_factor = rospy.ServiceProxy('/blam/blam_slam/add_factor', AddFactor)
    if add_factor(key_from, key_to, quat[0], quat[1], quat[2], quat[3]).success:
        print('Successfully added a factor between %i and %i to the graph.' % (key_from, key_to))
    else:
        print('An error occurred while trying to add a factor between %i and %i.' % (key_from, key_to))

if __name__ == '__main__':
    try:
        if len(sys.argv) not in (3, 6, 8) or "h" in sys.argv[1].lower():
            print("""Usage:
    python {arg0} <from_key> <to_key>
        Creates a loop closure between keys from_key and to_key with a null transformation.

    python {arg0} <from_key> <to_key> <yaw> <pitch> <roll>
        Creates a loop closure between keys from_key and to_key with a rotation defined by
        the given yaw, pitch and roll angles in radians.

    python {arg0} <from_key> <to_key> quat <w> <x> <y> <z>
        Creates a loop closure between keys from_key and to_key with a rotation defined by
        the given quaternion coordinates w, x, y, z in radians.

    python {arg0} <from_key> <to_key> axis <angle> <x> <y> <z>
        Creates a loop closure between keys from_key and to_key with a rotation defined by
        a rotation about axis x, y, z by an angle in radians. The axis coordinates need not
        be normalized.

    python {arg0} <from_key> <to_key> euler <a> <b> <c> [spec=\"szyx\"]
        Creates a loop closure between keys from_key and to_key with a rotation defined by
        the given Euler angles about axes a, b, c in radians following the angle specification.
        This specication is a string that follows the convention in http://matthew-brett.github.io/transforms3d/reference/transforms3d.euler.html#specifying-angle-conventions
            By default, `spec` is defined as \"szyx\" which is the Tait-Bryan notation,
            also known as Cardan angles.
            Another common spec is \"rzxz\" which is outlined in http://mathworld.wolfram.com/EulerAngles.html
            """.format(arg0=sys.argv[0]))
        elif len(sys.argv) == 3:
            connect(int(sys.argv[1]), int(sys.argv[2]), [1, 0, 0, 0])
        elif len(sys.argv) == 6:
            import transforms3d
            yaw, pitch, roll = float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
            quat = transforms3d.euler.euler2quat(roll, pitch, yaw)
            connect(int(sys.argv[1]), int(sys.argv[2]), quat)
        elif len(sys.argv) == 8:
            if sys.argv[3].lower() == "quat":
                quat = [float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7])]
                connect(int(sys.argv[1]), int(sys.argv[2]), quat)
            elif sys.argv[3].lower() == "axis":
                import transforms3d
                angle, x, y, z = float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7])
                yaw, pitch, roll = transforms3d.euler.axangle2euler([x, y, z], angle)
                quat = transforms3d.euler.euler2quat(yaw, pitch, roll)
                connect(int(sys.argv[1]), int(sys.argv[2]), quat)
            elif sys.argv[3].lower() == "euler":
                import transforms3d
                yaw, pitch, roll = float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])
                quat = transforms3d.euler.euler2quat(yaw, pitch, roll, sys.argv[7])
                connect(int(sys.argv[1]), int(sys.argv[2]), quat)
            else:
                print("Unknown argument: \"%s\"" % sys.argv[3])

    except rospy.ROSInterruptException: pass
