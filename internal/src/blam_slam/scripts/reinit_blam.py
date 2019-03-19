#!/usr/bin/env python
"""Reinitialize BLAM with the given initial pose."""

import subprocess

import rospy
import tf
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import (PoseStamped, TransformStamped,\
                               PoseWithCovarianceStamped)


class ReinitBlam(object):
    
    def __init__(self):
        self.init_pos = rospy.Subscriber('set_pose',
                                         PoseWithCovarianceStamped,
                                         self.init_pos_cb,
                                         queue_size=10)

        self.robot_namespace = rospy.get_namespace().replace('/', '')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.blam_frame = rospy.get_param('~blam_frame', 'blam')
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

	#Set the initial transform to identity
        tf_msg = TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.blam_frame
        tf_msg.transform.rotation.w = 1
  	self.tf_broadcaster.sendTransform(tf_msg)      

    def stop_blam(self):
        """Send shutdown signal to BLAM node."""
	subprocess.call('rosnode kill /{}/blam_slam'.format(self.robot_namespace), shell=True)

    def start_blam(self):
        """Start BLAM with node_manger interface"""
	subprocess.call("rosservice call /{0}/default_localizer_blam/run \"node: '/{0}/blam_slam'\"".format(self.robot_namespace), shell=True)

    def init_pos_cb(self, pose_cov_msg):
        # Transform original msg pose to map frame
        pose_msg = PoseStamped(header=pose_cov_msg.header, pose=pose_cov_msg.pose.pose)
        pose_msg = self.tf_buf.transform(pose_msg, self.map_frame, timeout=rospy.Duration(3))
        tf_msg = self.convert_pose_to_transform(pose_msg)

	# Kill BLAM node
        self.stop_blam()

        # Publish map to blam TF
  	self.tf_broadcaster.sendTransform(tf_msg)      

	# reinitialize BLAM node
        self.start_blam()

    def convert_pose_to_transform(self, pose_msg):
        tf_msg = TransformStamped()
        tf_msg.header = pose_msg.header
        tf_msg.child_frame_id = self.blam_frame
        tf_msg.transform.translation.x = pose_msg.pose.position.x
        tf_msg.transform.translation.y = pose_msg.pose.position.y
        tf_msg.transform.translation.z = pose_msg.pose.position.z
        tf_msg.transform.rotation.x = pose_msg.pose.orientation.x
        tf_msg.transform.rotation.y = pose_msg.pose.orientation.y
        tf_msg.transform.rotation.z = pose_msg.pose.orientation.z
        tf_msg.transform.rotation.w = pose_msg.pose.orientation.w
        return tf_msg


def main():
    rospy.init_node("reinit_blam")

    conv = ReinitBlam()
    rospy.spin()


if __name__ == '__main__':
    main()
