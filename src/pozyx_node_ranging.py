#!/usr/bin/env python
import rospy
import socket
from geometry_msgs.msg import PoseStamped
from pozyx_msgs.msg import DeviceRangeArray
from pozyx_msgs.msg import DeviceRange as Range
from std_msgs.msg import String
from pypozyx import *
from pypozyx.tools.device_list import *
from pypozyx.tools.discovery import *
from pypozyx.tools.version_check import *
from tf.transformations import quaternion_from_euler
from math import radians
from visualization_msgs.msg import Marker

class pozyx_node(object):
    def __init__(self):
        super(pozyx_node, self).__init__()
	self.hostname = socket.gethostname()
	print self.hostname
        self.anchors = rospy.get_param("~anchors")
        print self.anchors
        self.pub_poses = rospy.Publisher(self.hostname+rospy.get_name()+'/pozyx_pose', PoseStamped, queue_size=1)

	# Pub Ranging
	self.pub_ranges = rospy.Publisher(self.hostname+rospy.get_name()+'/pozyx_range', DeviceRangeArray, queue_size=2)

        self.pozyx = PozyxSerial(get_first_pozyx_serial_port())
        self.pozyx.printDeviceInfo()
        self.setup_anchors()

    def setup_anchors(self):
        #adding devices
        status = self.pozyx.clearDevices()
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(DeviceCoordinates(anchor['anchor_id'], 1, Coordinates(anchor['px'], anchor['py'], anchor['pz'])))
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(PozyxConstants.ANCHOR_SELECT_AUTO, len(self.anchors))
        if status == POZYX_SUCCESS:
            self.pozyx.printDeviceList()
        else:
            self.printErrorCode()

    def do_positioning(self):
        while not rospy.is_shutdown():
            position = Coordinates()
            orientation = EulerAngles()
            status = self.pozyx.doPositioning(position, dimension=PozyxConstants.DIMENSION_3D, algorithm=PozyxConstants.POSITIONING_ALGORITHM_TRACKING, remote_id = None)
            status &= self.pozyx.getEulerAngles_deg(orientation, remote_id = None)

            if status == POZYX_SUCCESS: # if get pose from pozyx
                #print "X:", position.x, ", Y:", position.y, "Z:", position.z
                #print "Orientation:", str(orientation)
                print "Translation:", str(position)
                tag_pose = PoseStamped()
                tag_pose.header.stamp = rospy.Time.now()
                tag_pose.pose.position.x = position.x
                tag_pose.pose.position.y = position.y
                tag_pose.pose.position.z = position.z
                # print tag_pose.pose.position
                rot = quaternion_from_euler(-radians(orientation.pitch), radians(orientation.roll), -radians(orientation.heading))
                tag_pose.pose.orientation.x = rot[0]
                tag_pose.pose.orientation.y = rot[1]
                tag_pose.pose.orientation.z = rot[2]
                tag_pose.pose.orientation.w = rot[3]
                self.pub_poses.publish(tag_pose)

            if status == POZYX_FAILURE:
                self.printErrorCode()

	    ranges = DeviceRangeArray()
	    ranges.ranging = []
            for anchor in self.anchors:
                device_range = DeviceRange()
                status = self.pozyx.doRanging(anchor['anchor_id'], device_range)
                if status == POZYX_SUCCESS:
		    r = Range()
		    r.ID = anchor['anchor_id']
                    r.time = rospy.Time.now()
		    r.range = device_range[1]
                    ranges.ranging.append(r)
                    ranges.length += 1
                else:
                    self.printErrorCode()
            self.pub_ranges.publish(ranges)

    def printErrorCode(self):
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code)
        if status == POZYX_SUCCESS:
            print self.pozyx.getErrorMessage(error_code)

if __name__ == '__main__':
    rospy.init_node('pozyx_node',anonymous=False)
    pozyx_node = pozyx_node()

    try:
        pozyx_node.do_positioning()
    except rospy.ROSInterruptException:
        pass
