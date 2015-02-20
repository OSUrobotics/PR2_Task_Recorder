#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import atexit
import datetime
import sys

class RecordJoints(object):
    def __init__(self, filename):
        self.file = open(filename, 'w')
        self.joints_data = dict()
        rospy.init_node('record_joints')

        rospy.Subscriber("joint_states", JointState, self.record_callback)

    def record_callback(self,data):
        for i in range(0, len(data.name)):
            if data.name[i] not in self.joints_data.keys():
                self.joints_data[data.name[i]] = dict()
                self.joints_data[data.name[i]]['position'] = list()
                self.joints_data[data.name[i]]['velocity'] = list()
                self.joints_data[data.name[i]]['effort'] = list()
            self.joints_data[data.name[i]]['position'].append(data.position[i])
            self.joints_data[data.name[i]]['velocity'].append(data.velocity[i])
            self.joints_data[data.name[i]]['effort'].append(data.effort[i])

    def export(self):
            self.file.write(str(self.joints_data))
            self.joints_data.clear()
            self.file.close()  
                  
if __name__ == '__main__':
    save_loc = datetime.datetime.today().strftime("%d.%m.%y:%H.%M")
    if len(sys.argv) == 2:
        save_loc = sys.argv[1]

    print 'Printing to: ' + save_loc
    record_joints = RecordJoints(save_loc)
    atexit.register(record_joints.export)
    rospy.spin()
