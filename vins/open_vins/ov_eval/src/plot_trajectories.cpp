/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Eigen/Eigen>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <fstream>
#include <iostream>
#include <string>

#include "alignment/AlignTrajectory.h"
#include "alignment/AlignUtils.h"
#include "utils/Loader.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"


#include <Eigen/Geometry>
#include <cmath> // for degree conversion


#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

// Will plot the xy 3d position of the pose trajectories
void plot_xy_positions(const std::string &name, const std::string &color, const std::vector<Eigen::Matrix<double, 7, 1>> &poses) {

  // Paramters for our line
  std::map<std::string, std::string> params;
  params.insert({"label", name});
  params.insert({"linestyle", "-"});
  params.insert({"color", color});

  // Create vectors of our x and y axis
  std::vector<double> x, y;
  for (size_t i = 0; i < poses.size(); i++) {
    x.push_back(poses.at(i)(0));
    y.push_back(poses.at(i)(1));
  }

  // Finally plot
  matplotlibcpp::plot(x, y, params);
}

// Will plot the z 3d position of the pose trajectories
void plot_z_positions(const std::string &name, const std::string &color, const std::vector<double> &times,
                      const std::vector<Eigen::Matrix<double, 7, 1>> &poses) {

  // Paramters for our line
  std::map<std::string, std::string> params;
  params.insert({"label", name});
  params.insert({"linestyle", "-"});
  params.insert({"color", color});

  // Create vectors of our x and y axis
  std::vector<double> time, z;
  for (size_t i = 0; i < poses.size(); i++) {
    time.push_back(times.at(i));
    z.push_back(poses.at(i)(2));
  }
  // Finally plot
  matplotlibcpp::plot(time, z, params);
}


void plot_x_positions(const std::string &name, const std::string &color, const std::vector<double> &times,
                      const std::vector<Eigen::Matrix<double, 7, 1>> &poses) {

  // Paramters for our line
  std::map<std::string, std::string> params;
  params.insert({"label", name});
  params.insert({"linestyle", "-"});
  params.insert({"color", color});

  // Create vectors of our x and y axis
  std::vector<double> time, x;
  for (size_t i = 0; i < poses.size(); i++) {
    time.push_back(times.at(i));
    x.push_back(poses.at(i)(0));
  }
  // Finally plot
  matplotlibcpp::plot(time, x, params);
}


void plot_y_positions(const std::string &name, const std::string &color, const std::vector<double> &times,
                      const std::vector<Eigen::Matrix<double, 7, 1>> &poses) {

  // Paramters for our line
  std::map<std::string, std::string> params;
  params.insert({"label", name});
  params.insert({"linestyle", "-"});
  params.insert({"color", color});

  // Create vectors of our x and y axis
  std::vector<double> time, y;
  for (size_t i = 0; i < poses.size(); i++) {
    time.push_back(times.at(i));
    y.push_back(poses.at(i)(1));
  }
  // Finally plot
  matplotlibcpp::plot(time, y, params);
}



void plot_rotations(const std::string &name, const std::string &color, const std::vector<double> &times,
                           const std::vector<Eigen::Matrix<double, 7, 1>> &poses) {

  // Parameters for our line
  std::map<std::string, std::string> params;
  params.insert({"label", name});
  params.insert({"linestyle", "-"});
  params.insert({"color", color});

  // Create vectors for time, roll, pitch, and yaw
  std::vector<double> time, roll, pitch, yaw;

  for (size_t i = 0; i < poses.size(); i++) {
    time.push_back(times.at(i));

   
 // Extract quaternion from the pose
    Eigen::Quaterniond quat(poses.at(i).segment<4>(3)); // Assuming quaternion is stored from index 3 to 6

    // Directly convert quaternion to Euler angles
    double roll_angle = std::atan2(2.0 * (quat.w() * quat.x() + quat.y() * quat.z()), 1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y()));
    double pitch_angle = std::asin(2.0 * (quat.w() * quat.y() - quat.z() * quat.x()));
    double yaw_angle = std::atan2(2.0 * (quat.w() * quat.z() + quat.x() * quat.y()), 1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));

 // Convert radians to degrees
    roll.push_back(roll_angle * (180.0 / M_PI));
    pitch.push_back(pitch_angle * (180.0 / M_PI));
    yaw.push_back(yaw_angle * (180.0 / M_PI));


    // // Convert quaternion to Euler angles (in radians)
    // Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX rotation order

    // Convert radians to degrees
    // roll.push_back(std::fmod(euler(2) * (180.0 / M_PI) + 360.0, 360.0));
    // pitch.push_back(std::fmod(euler(1) * (180.0 / M_PI) + 360.0, 360.0));
    // yaw.push_back(std::fmod(euler(0) * (180.0 / M_PI) + 360.0, 360.0));
  }

  // Plot roll
  matplotlibcpp::subplot(3, 1, 1);
  matplotlibcpp::plot(time, roll, params);
  matplotlibcpp::ylabel("Roll (degrees)");
    matplotlibcpp::grid(true);

  // Plot pitch
  matplotlibcpp::subplot(3, 1, 2);
  matplotlibcpp::plot(time, pitch, params);
  matplotlibcpp::ylabel("Pitch (degrees)");
    matplotlibcpp::grid(true);

  // Plot yaw
  matplotlibcpp::subplot(3, 1, 3);
  matplotlibcpp::plot(time, yaw, params);
  matplotlibcpp::xlabel("timestamp (sec)");
  matplotlibcpp::ylabel("Yaw (degrees)");
    matplotlibcpp::grid(true);

  // Show the subplots
  // matplotlibcpp::show();
}






#endif

int main(int argc, char **argv) {

  // Verbosity setting
  ov_core::Printer::setPrintLevel("INFO");

  // Ensure we have a path
  if (argc < 3) {
    PRINT_ERROR(RED "ERROR: Please specify a align mode and trajectory file\n" RESET);
    PRINT_ERROR(RED "ERROR: ./plot_trajectories <align_mode> <file_gt.txt> <file_est1.txt> ...  <file_est9.txt>\n" RESET);
    PRINT_ERROR(RED "ERROR: rosrun ov_eval plot_trajectories <align_mode> <file_gt.txt> <file_est1.txt> ...  <file_est9.txt>\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Read in all our trajectories from file
  std::vector<std::string> names;
  std::vector<std::vector<double>> times;
  std::vector<std::vector<Eigen::Matrix<double, 7, 1>>> poses;
  for (int i = 2; i < argc; i++) {

    // Read in trajectory data
    std::vector<double> times_temp;
    std::vector<Eigen::Matrix<double, 7, 1>> poses_temp;
    std::vector<Eigen::Matrix3d> cov_ori_temp, cov_pos_temp;
    ov_eval::Loader::load_data(argv[i], times_temp, poses_temp, cov_ori_temp, cov_pos_temp);

    // Align all the non-groundtruth trajectories to the base one
    if (i > 2) {

      // Intersect timestamps
      std::vector<double> gt_times_temp(times.at(0));
      std::vector<Eigen::Matrix<double, 7, 1>> gt_poses_temp(poses.at(0));
      ov_eval::AlignUtils::perform_association(0, 0.02, times_temp, gt_times_temp, poses_temp, gt_poses_temp);

      // Return failure if we didn't have any common timestamps
      if (poses_temp.size() < 3) {
        PRINT_ERROR(RED "[TRAJ]: unable to get enough common timestamps between trajectories.\n" RESET);
        PRINT_ERROR(RED "[TRAJ]: does the estimated trajectory publish the rosbag timestamps??\n" RESET);
        std::exit(EXIT_FAILURE);
      }

      // Perform alignment of the trajectories
      Eigen::Matrix3d R_ESTtoGT;
      Eigen::Vector3d t_ESTinGT;
      double s_ESTtoGT;
      ov_eval::AlignTrajectory::align_trajectory(poses_temp, gt_poses_temp, R_ESTtoGT, t_ESTinGT, s_ESTtoGT, argv[1]);

      // Debug print to the user
      Eigen::Vector4d q_ESTtoGT = ov_core::rot_2_quat(R_ESTtoGT);
      PRINT_DEBUG("[TRAJ]: q_ESTtoGT = %.3f, %.3f, %.3f, %.3f | p_ESTinGT = %.3f, %.3f, %.3f | s = %.2f\n", q_ESTtoGT(0), q_ESTtoGT(1),
                  q_ESTtoGT(2), q_ESTtoGT(3), t_ESTinGT(0), t_ESTinGT(1), t_ESTinGT(2), s_ESTtoGT);

      // Finally lets calculate the aligned trajectories
      std::vector<Eigen::Matrix<double, 7, 1>> est_poses_aignedtoGT;
      for (size_t j = 0; j < gt_times_temp.size(); j++) {
        Eigen::Matrix<double, 7, 1> pose_ESTinGT;
        pose_ESTinGT.block(0, 0, 3, 1) = s_ESTtoGT * R_ESTtoGT * poses_temp.at(j).block(0, 0, 3, 1) + t_ESTinGT;
        pose_ESTinGT.block(3, 0, 4, 1) = ov_core::quat_multiply(poses_temp.at(j).block(3, 0, 4, 1), ov_core::Inv(q_ESTtoGT));
        est_poses_aignedtoGT.push_back(pose_ESTinGT);
      }

      // Overwrite our poses with the corrected ones
      poses_temp = est_poses_aignedtoGT;
    }

    // Debug print the length stats
    boost::filesystem::path path(argv[i]);
    std::string name = path.stem().string();
    double length = ov_eval::Loader::get_total_length(poses_temp);
    PRINT_INFO("[COMP]: %d poses in %s => length of %.2f meters\n", (int)times_temp.size(), name.c_str(), length);

    // Save this to our arrays
    names.push_back(name);
    times.push_back(times_temp);
    poses.push_back(poses_temp);
  }

#ifdef HAVE_PYTHONLIBS

  // Colors that we are plotting
    // std::vector<std::string> colors = {"orange", "blue", "red", "black", "green", "cyan", "magenta"}; // color for model crane
    std::vector<std::string> colors = {"blue", "red", "green", "cyan", "magenta"}; // color for real crane kobleco
  // assert(algo_rpe.size() <= colors.size()*linestyle.size());

  // Plot this figure
  matplotlibcpp::figure_size(1000, 750);

  // Plot the position trajectories
  for (size_t i = 0; i < times.size(); i++) {
    plot_xy_positions(names.at(i), colors.at(i), poses.at(i));
  }

  // Display to the user
  matplotlibcpp::xlabel("x-axis (m)");
  matplotlibcpp::ylabel("y-axis (m)");
  matplotlibcpp::tight_layout();
  matplotlibcpp::show(false);



  // Plot this figure
  //======================================
  matplotlibcpp::figure_size(600, 600);

  // Zero our time arrays
  double starttime = (times.at(0).empty()) ? 0 : times.at(0).at(0);
  double endtime = (times.at(0).empty()) ? 0 : times.at(0).at(times.at(0).size() - 1);
  for (size_t i = 0; i < times.size(); i++) {
    for (size_t j = 0; j < times.at(i).size(); j++) {
      times.at(i).at(j) -= starttime;
    }
  }

  // Plot the position trajectories
  matplotlibcpp::subplot(3, 1, 1);
  for (size_t i = 0; i < times.size(); i++) {
    plot_x_positions(names.at(i), colors.at(i), times.at(i), poses.at(i));
  }
  matplotlibcpp::ylabel("tx (m)");
    matplotlibcpp::grid(true);

    matplotlibcpp::subplot(3, 1, 2);
  for (size_t i = 0; i < times.size(); i++) {
    plot_y_positions(names.at(i), colors.at(i), times.at(i), poses.at(i));
  }
   matplotlibcpp::ylabel("ty (m)");
    matplotlibcpp::grid(true);

    matplotlibcpp::subplot(3, 1, 3);
  for (size_t i = 0; i < times.size(); i++) {
    plot_z_positions(names.at(i), colors.at(i), times.at(i), poses.at(i));
  }

    // Display to the user
  matplotlibcpp::xlabel("timestamp (sec)");
  matplotlibcpp::ylabel("tz (m)");
      matplotlibcpp::grid(true);

  // matplotlibcpp::xlim(0.0, endtime - starttime);
  matplotlibcpp::legend();
  matplotlibcpp::tight_layout();


  // Create a subplot with 3 rows and 1 column
    matplotlibcpp::figure_size(600, 600);
  // matplotlibcpp::subplot(3, 1);
  for (size_t i = 0; i < times.size(); i++) {
    plot_rotations(names.at(i), colors.at(i), times.at(i), poses.at(i));
  }
  // matplotlibcpp::xlim(0.0, endtime - starttime);
  matplotlibcpp::legend();
  matplotlibcpp::tight_layout();
    matplotlibcpp::grid(true);



  matplotlibcpp::show(true);

#endif

  // Done!
  return EXIT_SUCCESS;
}
