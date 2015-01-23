#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import atexit
class RecordJoints(object):
    def __init__(self):
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
            f = open('workfile', 'a')
            f.write(str(self.joints_data))
            self.joints_data.clear()
            f.close()        
if __name__ == '__main__':
    record_joints = RecordJoints()
    atexit.register(record_joints.export)
    rospy.spin()
