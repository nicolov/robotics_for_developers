/*
 * @file ros_gtsam_interop.h
 * @brief Utility functions for GTSAM/ROS interop
 */

#pragma once

#include <gtsam/geometry/Cal3DS2.h>

#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include <rcars_detector_msgs/Tag.h>

namespace fiducial_slam {

    /// Convert a ROS camera info message to a GTSAM calibration
    boost::shared_ptr<gtsam::Cal3DS2> caminfo_to_gtsam_calib(const sensor_msgs::CameraInfoConstPtr &m,
                                                             const bool use_distortion = false) {
        /* rcars_detector uses the rectified image, so we should ignore the distorsion coefficients
         * in the CameraInfo message.
         */
        boost::shared_ptr<gtsam::Cal3DS2> K;
        if (use_distortion) {
            K.reset(new gtsam::Cal3DS2(m->K[0], m->K[4], 0, m->K[2], m->K[5],
                                       m->D[0], m->D[1], m->D[2], m->D[3]));
        } else {
            K.reset(new gtsam::Cal3DS2(m->K[0], m->K[4], 0, m->K[2], m->K[5],
                                       0.0, 0.0, 0.0, 0.0));
        }
        return K;
    }

    std::array<gtsam::Point2, 4> tag_to_gtsam_points(const rcars_detector_msgs::Tag &m) {
        std::array<gtsam::Point2, 4> corners;
        for (int i = 0; i < 4; ++i) {
            corners[i] = gtsam::Point2(m.corners[i].x, m.corners[i].y);
        }

        return corners;
    };

    /// Convert a ROS geometry_msgs/Pose message to a GTSAM pose
    gtsam::Pose3 gmsgs_pose_to_gtsam(const geometry_msgs::Pose &p) {
        gtsam::Point3 point(p.position.x, p.position.y, p.position.z);
        auto rot = gtsam::Rot3::Quaternion(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
        return gtsam::Pose3(rot, point);
    }

    geometry_msgs::Pose gtsam_pose_to_gmsgs(const gtsam::Pose3 &pose) {
        geometry_msgs::Pose p;
        tf::pointEigenToMsg(pose.translation().vector(), p.position);
        tf::quaternionEigenToMsg(pose.rotation().toQuaternion(), p.orientation);
        return p;
    }

    geometry_msgs::Vector3 gtsam_vector_to_gmsgs(const gtsam::Vector3 &vec) {
        geometry_msgs::Vector3 v;
        tf::vectorEigenToMsg(vec, v);
        return v;
    }

    /// Convert a ROS geometry_msgs/Transform message to a GTSAM pose
    gtsam::Pose3 gmsgs_transf_to_gtsam(const geometry_msgs::Transform &transf) {
        geometry_msgs::Pose pose;
        pose.orientation = transf.rotation;
        Eigen::Vector3d t;
        tf::vectorMsgToEigen(transf.translation, t);
        tf::pointEigenToMsg(t, pose.position);

        return gmsgs_pose_to_gtsam(pose);
    }

}
