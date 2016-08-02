/*
 * @file gtsam_opencv_interop.h
 * @brief Utility functions for GTSAM/OpenCV interop
 */

#pragma once

#include <utility>

#include <opencv/cv.h>
#include <opencv2/core/eigen.hpp>

#include <gtsam/geometry/Cal3_S2.h>

namespace fiducial_slam {

    /// GTSAM-style camera calibration to OpenCV projection and distorsion matrices
    std::pair<cv::Mat, cv::Mat> gtsam_calib_to_cv(const boost::shared_ptr<gtsam::Cal3DS2> K_) {
        cv::Mat proj;
        cv::eigen2cv(K_->K(), proj);
        cv::Mat dist;
        cv::eigen2cv(K_->k(), dist);
        return std::make_pair(proj, dist);
    };

}