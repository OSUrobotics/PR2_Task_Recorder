import numpy as np
from sklearn.decomposition import PCA
import operator

names = ['fl_caster_rotation_joint', 'fl_caster_l_wheel_joint', 'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint', 'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint', 'bl_caster_rotation_joint', 'bl_caster_l_wheel_joint', 'bl_caster_r_wheel_joint', 'br_caster_rotation_joint', 'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint', 'torso_lift_joint', 'torso_lift_motor_screw_joint', 'head_pan_joint', 'head_tilt_joint', 'laser_tilt_mount_joint', 'r_upper_arm_roll_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_forearm_roll_joint', 'r_elbow_flex_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint', 'r_gripper_r_finger_tip_joint', 'r_gripper_l_finger_tip_joint', 'r_gripper_motor_screw_joint', 'r_gripper_motor_slider_joint', 'l_upper_arm_roll_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'l_gripper_joint', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'l_gripper_r_finger_tip_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_motor_screw_joint', 'l_gripper_motor_slider_joint']

names_subsampled = ['head_pan_joint', 'head_tilt_joint', 'r_upper_arm_roll_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_forearm_roll_joint', 'r_elbow_flex_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint', 'r_gripper_r_finger_tip_joint', 'r_gripper_l_finger_tip_joint', 'r_gripper_motor_screw_joint', 'r_gripper_motor_slider_joint', 'l_upper_arm_roll_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'l_gripper_joint', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'l_gripper_r_finger_tip_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_motor_screw_joint', 'l_gripper_motor_slider_joint']
file_data = open('workfile', 'r').read()
data = eval(file_data)

# X = list()
# for i in range(0, len(data['r_shoulder_pan_joint']['position'])):
# 	X_temp = list()
# 	for ele2 in names_subsampled:
# 		X_temp.append(data[ele2]['effort'][i])
# 	X.append(X_temp)

# X = np.asarray(X)

# #print X
# pca = PCA(n_components=len(names_subsampled))
# pca.fit(X)
# #print pca.score_samples(X)

# print pca.components_
# print pca.explained_variance_ratio_

# weights = [0] * len(names_subsampled)

# weights_dict = dict()

# for i in range(0,len(pca.explained_variance_ratio_)):
# 	for j in range(0,len(pca.components_[i])):
# 		if names_subsampled[j] not in weights_dict.keys():
# 			weights_dict[names_subsampled[j]] = 0
# 		weights[j] += abs(pca.explained_variance_ratio_[i]) * abs(pca.components_[i][j])
# 		weights_dict[names_subsampled[j]] += abs(pca.explained_variance_ratio_[i]) * abs(pca.components_[i][j])
# sorted_x = sorted(weights_dict.items(), key=operator.itemgetter(1), reverse=True)
# print sorted_x
# print pca.noise_variance_


X2 = list()
for ele in names_subsampled:
	for i in range(0, len(data[ele]['position'])):
		X2.append([data[ele]['position'][i], data[ele]['velocity'][i], data[ele]['effort'][i]])

pca2 = PCA(n_components=3)
pca2.fit(X2)

print pca2.components_
print pca2.explained_variance_ratio_