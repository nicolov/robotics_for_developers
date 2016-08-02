/*
 * @file SingleMarkerLocalizer.h
 * @brief Camera localization using a single marker
 */

#pragma once

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include <r4d_common/MarkerFactor.h>
#include <r4d_common/ros_gtsam_interop.h>
#include <r4d_common/ros_opencv_interop.h>
#include <r4d_common/gtsam_opencv_interop.h>

namespace fiducial_slam {

    /// Robot localization using a single marker
    struct SingleMarkerLocalizer {
        SingleMarkerLocalizer(ros::NodeHandle &nh, ros::NodeHandle& nh_p) :
                nh_(nh),
                nh_p_(nh_p),
                tag_size_(0.16),
                pub_cv_(nh_.advertise<nav_msgs::Odometry>("/p_cv", 10)),
                pub_gtsam_(nh_.advertise<nav_msgs::Odometry>("/p_gtsam", 10)) {
        }

        void image_info_cb(const sensor_msgs::CameraInfoConstPtr &m) {
            if (!K_) {
                K_ = fiducial_slam::caminfo_to_gtsam_calib(m);
                K_->print("Calibration from camera_info msg:\n");
            }
        }

        void handle_external_pose(const geometry_msgs::Pose& ext_pose, const std_msgs::Header& header, std::string label) {
            auto pVC = gmsgs_pose_to_gtsam(ext_pose).inverse();

            // This is the first pair of measurements: determine the calibration
            // between ground truth and marker
            if (!ground_truth_calib_ && first_cam_pose_) {
                // MV = MC * VC^-1
                ground_truth_calib_ = (*first_cam_pose_) * pVC.inverse();
            }

            if (ground_truth_calib_) {
                // MC = MV * VC
                auto pMC = (*ground_truth_calib_) * pVC;

                tf::Transform tf_transf;
                tf::poseMsgToTF(gtsam_pose_to_gmsgs(pMC), tf_transf);
                tf_br_.sendTransform(
                        tf::StampedTransform(tf_transf, header.stamp, "world", label));
            }
        }

        void filter_output_cb(const nav_msgs::OdometryConstPtr &m) {
            handle_external_pose(m->pose.pose, m->header, "ekf");
        }

        void ground_truth_cb(const geometry_msgs::TransformStampedConstPtr &m) {
            geometry_msgs::Pose pose;
            Eigen::Vector3d t;
            tf::vectorMsgToEigen(m->transform.translation, t);
            tf::pointEigenToMsg(t, pose.position);
            pose.orientation = m->transform.rotation;

            //handle_external_pose(pose, m->header, "vicon");
        }

        geometry_msgs::Pose pose_using_cv(const rcars_detector_msgs::Tag &tag) {
            auto cv_calib = fiducial_slam::gtsam_calib_to_cv(K_);
            auto cv_corners = fiducial_slam::tag_to_cv_points(tag, tag_size_);
            return fiducial_slam::cv_marker_pose(cv_corners, cv_calib);
        }

        geometry_msgs::Pose pose_using_gtsam(const rcars_detector_msgs::Tag &tag) {
            gtsam::NonlinearFactorGraph graph;
            gtsam::Values estimate;

            auto pixel_noise = gtsam::noiseModel::Isotropic::shared_ptr(
                    gtsam::noiseModel::Isotropic::Sigma(8, 1));
            auto origin_noise = gtsam::noiseModel::Isotropic::shared_ptr(
                    gtsam::noiseModel::Isotropic::Sigma(6, 1e-3));

            auto s_ic = gtsam::Symbol('C', 0);
            auto s_marker = gtsam::Symbol('M', tag.id);

            estimate.insert(s_marker, gtsam::Pose3());
            auto guess = fiducial_slam::gmsgs_pose_to_gtsam(tag.pose).inverse();
            // Initialize off the "right" value
            auto guess_distorsion = gtsam::Pose3(
                gtsam::Rot3::Ypr(0.2, -0.2, 0.1),
                gtsam::Point3(0.1, -0.05, 0));
            estimate.insert(s_ic, guess * guess_distorsion);

            auto corners = fiducial_slam::tag_to_gtsam_points(tag);
            graph.push_back(fiducial_slam::MarkerFactor(corners, pixel_noise, s_ic, s_marker, K_, tag_size_));
            graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(s_marker, gtsam::Pose3(), origin_noise));

            auto result = gtsam::GaussNewtonOptimizer(graph, estimate).optimize();
            auto result_p_ic = result.at<gtsam::Pose3>(s_ic);

            // NOTE: inverse here to account for the passive/active interpretation
            return fiducial_slam::gtsam_pose_to_gmsgs(result_p_ic.inverse());
        }

        void tags_cb(const rcars_detector_msgs::TagArrayConstPtr msg) {
            // A couple of utility closures to publish data
            auto send_tf = [&](const geometry_msgs::Pose& p, std::string frame) {
                tf::Transform tf_transf;
                tf::poseMsgToTF(p, tf_transf);
                tf_br_.sendTransform(
                        tf::StampedTransform(tf_transf, msg->header.stamp, "world", frame));
            };

            auto pose_to_odometry = [&](const geometry_msgs::Pose& p) {
                nav_msgs::Odometry odom;
                odom.header = msg->header;
                odom.pose.pose = p;
                return odom;
            };

            if (msg->tags.size()) {
                if (!tracked_tag_id_) {
                    // Let's just start tracking the first tag we've ever seen
                    tracked_tag_id_ = msg->tags[0].id;
                    ROS_WARN_STREAM("Starting to track marker with id " << static_cast<int>(*tracked_tag_id_));
                }

                // Look for the tag we have been tracking
                auto tracked_it = std::find_if(msg->tags.begin(), msg->tags.end(), [&](const rcars_detector_msgs::Tag &tag) {
                    return tag.id == *tracked_tag_id_;
                });

                if (tracked_it != msg->tags.end()) {
                    auto &tracked_tag = *tracked_it;
                    auto corners = fiducial_slam::tag_to_gtsam_points(tracked_tag);

                    {
                        std::cout << "Pose by rcars_detector\n" << tracked_tag.pose << "\n";
                        send_tf(tracked_tag.pose, "dtct");
                    }

                    {
                        auto cv_pose = pose_using_cv(tracked_tag);
                        std::cout << "Pose by OpenCV\n" << cv_pose << "\n";

                        pub_cv_.publish(pose_to_odometry(cv_pose));
                        send_tf(cv_pose, "cv");
                    }

                    {
                        auto gtsam_pose = pose_using_gtsam(tracked_tag);
                        std::cout << "Pose by GTSAM\n" << gtsam_pose << "\n";

                        pub_gtsam_.publish(pose_to_odometry(gtsam_pose));
                        send_tf(gtsam_pose, "gtsam");

                        // If needed, set the first camera pose that will used to
                        // find the calibration w.r.t. the ground truth meas.
                        if (!first_cam_pose_) {
                            first_cam_pose_ = gmsgs_pose_to_gtsam(gtsam_pose);
                        }
                    }
                    std::cout << "\n\n";
                } else {
                    ROS_WARN_THROTTLE(1, "Tracked marker not found");
                }
            }
        }

        void imu_cb(const sensor_msgs::ImuConstPtr &m) {
        }

        boost::shared_ptr<gtsam::Cal3DS2> K_;
        double tag_size_;
        boost::optional<unsigned char> tracked_tag_id_; // Id of the (single) tag we are tracking
        boost::optional<gtsam::Pose3> first_cam_pose_;
        boost::optional<gtsam::Pose3> ground_truth_calib_;

        ros::NodeHandle nh_, nh_p_;
        tf::TransformBroadcaster tf_br_;

        ros::Publisher pub_cv_, pub_gtsam_;
    };
}