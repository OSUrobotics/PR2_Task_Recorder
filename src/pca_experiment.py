#!/usr/bin/env python
#import rospy
from os import listdir
import yaml
from rosbag.bag import Bag
import atexit
import re
import numpy as np
from sklearn.decomposition import PCA
import operator



class PCAProcessing(object):
	def __init__(self):
		self.names = ['fl_caster_rotation_joint', 'fl_caster_l_wheel_joint', 'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint', 'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint', 'bl_caster_rotation_joint', 'bl_caster_l_wheel_joint', 'bl_caster_r_wheel_joint', 'br_caster_rotation_joint', 'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint', 'torso_lift_joint', 'torso_lift_motor_screw_joint', 'head_pan_joint', 'head_tilt_joint', 'laser_tilt_mount_joint', 'r_upper_arm_roll_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_forearm_roll_joint', 'r_elbow_flex_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint', 'r_gripper_r_finger_tip_joint', 'r_gripper_l_finger_tip_joint', 'r_gripper_motor_screw_joint', 'r_gripper_motor_slider_joint', 'l_upper_arm_roll_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'l_gripper_joint', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'l_gripper_r_finger_tip_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_motor_screw_joint', 'l_gripper_motor_slider_joint']
		self.names_subsampled = ['r_upper_arm_roll_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_forearm_roll_joint', 'r_elbow_flex_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint', 'r_gripper_r_finger_tip_joint', 'r_gripper_l_finger_tip_joint', 'r_gripper_motor_screw_joint', 'r_gripper_motor_slider_joint', 'l_upper_arm_roll_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'l_gripper_joint', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'l_gripper_r_finger_tip_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_motor_screw_joint', 'l_gripper_motor_slider_joint']

	def obtain_data(self,filename):
		file_data = open(filename, 'r').read()
		data = eval(file_data)
		print('Read Data')
		return data
	

	def process_data_by_position(self, data):
		X = list()
		for i in range(0, len(data['r_shoulder_pan_joint']['position'])):
			X_temp = list()
			for j in self.names_subsampled:
				X_temp.append(data[j]['position'][i])
			X.append(X_temp)
		X = np.asarray(X)
		return X	

	def process_data_by_joint(self, data):
		X = list()
		for i in range(0, len(data['r_shoulder_pan_joint']['position'])):
			X_temp = list()
			for ele2 in self.names_subsampled:
				X_temp.append(data[ele2]['position'][i])
			X.append(X_temp)
		X = np.asarray(X)
		return X	
	def fit_pca_to_data(self, data, components):

		pca = PCA(n_components=components)
		pca.fit(data)
		print pca.explained_variance_ratio_
		print pca.components_

		#weights = [0] * len(self.names_subsampled)

		#weights_dict = dict()

		# for i in range(0,len(pca.explained_variance_ratio_)):
		# 	for j in range(0,len(pca.components_[i])):
		# 		if self.names_subsampled[j] not in weights_dict.keys():
		# 			weights_dict[self.names_subsampled[j]] = 0
		# 		weights[j] += abs(pca.explained_variance_ratio_[i]) * abs(pca.components_[i][j])
		# 		weights_dict[self.names_subsampled[j]] += abs(pca.explained_variance_ratio_[i]) * abs(pca.components_[i][j])
		# sorted_x = sorted(weights_dict.items(), key=operator.itemgetter(1), reverse=True)
		# print sorted_x


		# X2 = list()
		# for ele in names_subsampled:
		# 	for i in range(0, len(data[ele]['position'])):
		# 		X2.append([data[ele]['position'][i], data[ele]['velocity'][i], data[ele]['effort'][i]])

		# print 'running PCA'
		# pca2 = PCA(n_components=3)
		# pca2.fit(X2)

		# print pca2.components_
		# print pca2.explained_variance_ratio_
		return pca


	def inverse_transform_pca(self, data):
		return pca.inverse_transform(data)

	def transform_pca(self, data):
		return pca.transform(data)

	def output_to_dictionary(self, data, filename):
		print 'hi'
		output = dict()
		for it in range(0,len(data)):
			vec = data[it]
			for it_vec in range(0,len(vec)):
				if self.names_subsampled[it_vec] not in output.keys():
					output[self.names_subsampled[it_vec]] = list()
				output[self.names_subsampled[it_vec]].append(vec[it_vec])

		open_file = open(filename, 'w')
		open_file.write(str(output))
if __name__ == '__main__':




	experiment = PCAProcessing()
	data = experiment.obtain_data('initial_test')
	processed_data = experiment.process_data_by_joint(data)
	pca = experiment.fit_pca_to_data(processed_data, 3)


	test_data = experiment.obtain_data("11.02.15:16.44")
	processed_test_data = experiment.process_data_by_position(test_data)
	transformed_data = experiment.transform_pca(processed_test_data)
	transformed_back_data = experiment.inverse_transform_pca(transformed_data)
	experiment.output_to_dictionary(transformed_back_data, 'manifolded_trajectory')
	experiment.output_to_dictionary(processed_test_data, 'original_trajectory')








	
	# val = 0
	# errors_by_joint = [0] * len(processed_test_data[0])
	# for it in range(0,len(processed_test_data)):
	# 	vec_original = processed_test_data[it]
	# 	vec_transformed = transformed_back_data[it]
	# 	for it_vec in range(0, len(vec_original)):
	# 		errors_by_joint[it_vec] += abs(vec_original[it_vec]-vec_transformed[it_vec])

	# for e in range(0, len(errors_by_joint)):
	# 	errors_by_joint[e] = errors_by_joint[e]/len(processed_test_data)
	

	# print errors_by_joint

	# errors_by_position = [0] * len(processed_test_data)
	# for it in range(0,len(processed_test_data)):
	# 	vec_original = processed_test_data[it]
	# 	vec_transformed = transformed_back_data[it]
	# 	for it_vec in range(0, len(vec_original)):
	# 		errors_by_position[it] += abs(vec_original[it_vec]-vec_transformed[it_vec])

	# errors_by_position_mean_total = 0
	# for e in range(0, len(errors_by_position)):
	# 	errors_by_position_mean_total += errors_by_position[e]/len(processed_test_data)

	# print errors_by_position_mean_total
	# for e in range(0, len(errors_by_position)):
	# 	errors_by_position[e] = errors_by_position[e]/len(processed_test_data[0])

	# #print errors_by_position

	# exit()	

	# print processed_test_data[0]
	
	# print transformed_back_data[0]

