import rospy
import numpy
from pozyx_msgs.msg import DeviceRangeArray
from pozyx_msgs.msg import DeviceRange as Range
from pypozyx import *

truth = 80000
anchor_id = '0x6a2c'
rate = 10

class pozyx_node():
    def __init__(self):
        self.pub_range = rospy.Publisher('~/pozyx_range', DeviceRangeArray, queue_size = 1)
        self.pozyx = PozyxSerial(get_first_pozyx_serial_port())
        self.pozyx.printDeviceInfo()
        self.rate = rospy.Rate(rate)
        self.measurements = []
        self.n = 100;

    def do_Range(self):
        while self.n:
            ranges = DeviceRangeArray()
            ranges.ranging = []
            device_range = DeviceRange()
            status = self.pozyx.doRanging(anchor_id, device_range)
            if status == POZYX_SUCCESS:
                r = Range()
                r.ID = int(anchor_id, 16)
                r.time = rospy.Time.now()
                r.range = device_range[1]
                self.measurements.append(r.range)
                ranges.ranging.append(r)
                ranges.length += 1
            else:
                self.printErrorCode()
            self.pub_range.publish(ranges)
            rospy.loginfo(ranges)
            self.n -= 1
            self.rate.sleep()
        print(self.measurements)
        self.error_calc()

    def error_calc(self):
        errors = [abs(x - truth) for x in self.measurements]
        mean_error = sum(errors)/len(errors)
        std = numpy.std(self.measurements)
        print(mean_error, std);


    def printErrorCode(self):
        error_code = SingleRegister()
        if self.pozyx.getErrorCode(error_code) == POZYX_SUCCESS:
            print self.pozyx.getErrorMessage(error_code)

if __name__ == '__main__':
    rospy.init_node('pozyx_node', anonymous=False)
    pozyx_node = pozyx_node()

    try:
        pozyx_node.do_Range()
    except rospy.ROSInterruptException:
        pass
