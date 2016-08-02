/*
 * @file ros_opencv_interop.h
 * @brief Utility functions for ROS/OpenCV interop
 */

#pragma once

#include <utility>
#include <opencv2/calib3d.hpp>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>

namespace fiducial_slam {
    using cv_corners_t = std::pair<std::vector<cv::Point2f>, std::vector<cv::Point3f>>;

    /// Convert a Tag message to OpenCV points ready for solvePnP
    cv_corners_t tag_to_cv_points(const rcars_detector_msgs::Tag &m,
                                  double tag_size) {
        std::vector<cv::Point2f> image_points;
        std::vector<cv::Point3f> world_points;

        const double half = 0.5 * tag_size;

        for (int i = 0; i < 4; ++i) {
            image_points.emplace_back(m.corners[i].x, m.corners[i].y);
        }

        world_points.emplace_back(-half,  half, 0);  // top left
        world_points.emplace_back( half,  half, 0);  // top right
        world_points.emplace_back(-half, -half, 0);  // bottom left
        world_points.emplace_back( half, -half, 0);  // bottom right

        auto ret = std::make_pair(image_points, world_points);
        return ret;
    }

    /// Find camera-marker pose using OpenCV
    geometry_msgs::Pose cv_marker_pose(cv_corners_t& corners,
                                   std::pair<cv::Mat, cv::Mat>& calibration) {
        auto& cv_proj = calibration.first;
        auto& cv_dist = calibration.second;

        cv::Mat cv_r, cv_t;
        cv::solvePnP(corners.second, corners.first, cv_proj, cv_dist, cv_r, cv_t);

        cv::Mat cv_r_mat;
        cv::Rodrigues(cv_r, cv_r_mat);

        Eigen::Vector3d translation;
        translation << cv_t.at<double>(0), cv_t.at<double>(1), cv_t.at<double>(2);
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> cv_rot_mat_eig(cv_r_mat.ptr<double>(), 3, 3);
        Eigen::Quaterniond rotation(cv_rot_mat_eig);

        geometry_msgs::Pose result;
        tf::pointEigenToMsg(translation, result.position);
        tf::quaternionEigenToMsg(rotation, result.orientation);

        return result;
    }
}
