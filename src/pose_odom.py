#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class visualizer(object):
    def __init__(self):
        self.pub_odom = rospy.Publisher("~pose_visualizer", Odometry, queue_size=10)
        self.sub_pose = rospy.Subscriber("/pozyx_node/pozyx_pose", PoseStamped, self.pose_cb, queue_size=1)
    def pose_cb(self, pose_msg):
        #print "pose cb"
        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = rospy.Time()
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = pose_msg.pose.position.x/1000
        odom_msg.pose.pose.position.y = pose_msg.pose.position.y/1000
        odom_msg.pose.pose.position.z = pose_msg.pose.position.z/1000

        odom_msg.pose.pose.orientation.x = pose_msg.pose.orientation.x
        odom_msg.pose.pose.orientation.y = pose_msg.pose.orientation.y
        odom_msg.pose.pose.orientation.z = pose_msg.pose.orientation.z
        odom_msg.pose.pose.orientation.w = pose_msg.pose.orientation.w

        self.pub_odom.publish(odom_msg)

if __name__ == '__main__':
	rospy.init_node('pose_visualization',anonymous=False)
	node = visualizer()
	rospy.spin()
