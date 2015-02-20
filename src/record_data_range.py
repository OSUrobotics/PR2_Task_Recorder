#!/usr/bin/env python
from record_data import RecordJoints
#import rospy
from os import listdir
import yaml
from rosbag.bag import Bag
import atexit
import re

class RecordDataRange(object):
	
	def __init__(self, path, save_path = 'test', threshold = 50):
		self.record_joints = RecordJoints(save_path)
		atexit.register(self.record_joints.export)
		self.sorted_bags = list()
		self.sort_trials(path)
		self.print_trials(threshold)
		

	def sort_trials(self, path):
		trials = listdir(path)
		bags = dict()
		for trial in trials:
			is_bag = re.search(".bag$", trial, flags=0)
			if not is_bag:
				continue
			info_dict = yaml.load(Bag(trial, 'r')._get_yaml_info())
			#print info_dict["duration"]
			bags[info_dict["duration"]] = trial

		for key in sorted(bags):
			self.sorted_bags.append((key, bags[key]))

	def print_trials(self, bound):
		for i in self.sorted_bags:
			time_taken = i[0]
			bag_str = i[1]
			bag = Bag(bag_str)
			if time_taken < bound:
				for topic, msg, t in bag.read_messages(topics=['joint_states']):
					#print msg.name
					self.record_joints.record_callback(msg)			

if __name__ == '__main__':
	RecordDataRange(".")
	exit()



