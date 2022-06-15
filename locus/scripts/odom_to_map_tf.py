#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import tf_conversions
from tf import TransformBroadcaster, transformations
from geometry_msgs.msg import PoseStamped
import argparse

class TFBaseLinkPublisher():
    def __init__(self, robot_ns):
        self.robot_ns = robot_ns
        self.sub = rospy.Subscriber("/"+robot_ns  +  "/locus/odometry" , Odometry, self.callback)
        self.path = Path()
        self.path_pub = rospy.Publisher("/"+ robot_ns  + "/path", Path, queue_size=10)

    def callback(self, data):
        rospy.loginfo(data.pose.pose.position)
        br = TransformBroadcaster()
        time = rospy.Time.now()
        br.sendTransform((data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z),
                         (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w),
                         time,
                         self.robot_ns + "/base_link",
                         self.robot_ns + "/map")

        pose = PoseStamped()
        self.path.header = data.header
        pose = PoseStamped()
        pose.header = data.header
        pose.pose = data.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Example usage: python sub_odom_publish_tf.py husky4')
    parser.add_argument('robot_name', type=str, help='pass robot name etc')
    args = parser.parse_args()
    rospy.init_node('pub_odom_tf_map')
    tf_base_link_publisher = TFBaseLinkPublisher(args.robot_name)
    tf_base_link_publisher.run()