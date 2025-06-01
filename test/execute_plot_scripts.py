#!/usr/bin/env python

import subprocess
import time
import sys

import os


# rosrun ov_eval plot_trajectories <align_mode> <file_gt.txt> ... <file_est9.txt>
# rosrun ov_eval error_singlerun <align_mode> <file_gt.txt> <file_est.txt>
# rosrun ov_eval error_dataset <align_mode> <file_gt.txt> <folder_algorithms>
# rosrun ov_eval error_comparison <align_mode> <folder_groundtruth> <folder_algorithms>


# file_paths = [
#     path + 'truth/' + file_common_name + '/trj_01.txt',
#     path + file_common_name + '/vins_mono/trj_01/vins_mono.txt',
#     path + file_common_name + '/vins_mono+EKF/trj_01/vins_mono+EKF.txt',
#     path + file_common_name + '/vins_fusion_mono/trj_01/vins_fusion_mono.txt',
#     path + file_common_name + '/vins_fusion_mono+EKF/trj_01/vins_fusion_mono+EKF.txt',
#     path + file_common_name + '/vins_fusion_stereo/trj_01/vins_fusion_stereo.txt',
#     path + file_common_name + '/vins_fusion_stereo+EKF/trj_01/vins_fusion_stereo+EKF.txt',
# ]


# -----------------------------------------
# table for combine trajectory comparasion
# -------------------------------------------
# file_common_name = '2023-06-20-random_23578'
# truth_folder_address = os.path.join(path, "truth_combine")
# folder_address = os.path.join(path, file_common_name)
# command2 = "rosrun ov_eval error_comparison posyaw {} {}".format(truth_folder_address, folder_address)
# ---------------------------------------------------------


file_common_name = 'random_3'
path = '/home/aisl2/test/'
test_plot = "test/trj_01/"
vins_mono_plot = "vins_mono/trj_01/"
truth_file_address = os.path.join(path, "truth", file_common_name, "trj_01.txt")
truth_folder_address = os.path.join(path, "truth", file_common_name)
folder_address = os.path.join(path, file_common_name)

test_file_address = os.path.join(path, file_common_name, test_plot, "test.txt")


print("==============================")
# table for combine
# file_common_name = '2023-06-20-random_23578'
# truth_folder_address = os.path.join(path, "truth_combine")

command1 = "rosrun ov_eval error_dataset none {} {}".format(truth_file_address, folder_address)
command2 = "rosrun ov_eval error_comparison none {} {}".format(truth_folder_address, folder_address)
# command3 = "rosrun ov_eval plot_trajectories posyaw {} {} {}".format(truth_file_address, test_file_address)
# command4 = "rosrun ov_eval error_singlerun posyaw {} {} {}".format(truth_file_address, test_file_address)


# command3 = " rosrun ov_eval plot_trajectories posyaw  /home/aisl2/catkin_ws/src/data/model_crane_data/23_9_21_model_crane_data_to_compare_different_methods/random_1/test/trj_01/test.txt     /home/aisl2/catkin_ws/src/data/model_crane_data/23_9_21_model_crane_data_to_compare_different_methods/truth/random_1/trj_01.txt "
# print(command3)

print("truth_file_address", truth_file_address)
print(" ")
print("folder_address", folder_address)
print(" ")
print("test_file_address", test_file_address)
print(" ")
# print(command1)
print(" ")

# command = "rosrun ov_eval error_dataset none /home/aisl2/catkin_ws/src/data/model_crane_data/real_sense/truth/2023-06-20-random_01/trj_01.txt /home/aisl2/catkin_ws/src/data/model_crane_data/real_sense/2023-06-20-random_01"
# command = " rosrun ov_eval plot_trajectories posyaw /home/aisl2/catkin_ws/src/data/model_crane_data/real_sense/truth/2023-06-20-random_01/trj_01.txt /home/aisl2/catkin_ws/src/data/model_crane_data/real_sense/2023-06-20-random_01/vins_mono/trj_01/vins_mono.txt /home/aisl2/catkin_ws/src/data/model_crane_data/real_sense/2023-06-20-random_01/vins_mono+EKF/trj_01/vins_mono+EKF.txt"
# command = " rosrun ov_eval error_singlerun posyaw /home/aisl2/catkin_ws/src/data/model_crane_data/real_sense/truth/2023-06-20-random_01/trj_01.txt /home/aisl2/catkin_ws/src/data/model_crane_data/real_sense/2023-06-20-random_01/vins_mono/trj_01/vins_mono.txt"


# Start the process
process1 = subprocess.Popen(command1, shell=True)


process2 = subprocess.Popen(command2, shell=True)

# process3 = subprocess.Popen(command3, shell=True)

# process3 = subprocess.Popen(command3, shell=True)

# Wait for user input to terminate
sys.stdin.readline()

# Stop the process
# process2.terminate()

process1.terminate()
