/*
 * @file MultimarkerLocalizer.h
 * @brief Camera localization using multiple markers
 */

#pragma once

#include <fstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Duration.h>
#include <nav_msgs/Odometry.h>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include <rcars_detector_msgs/TagArray.h>

#include <r4d_common/MarkerFactor.h>
#include <r4d_common/ros_gtsam_interop.h>
#include <r4d_common/ros_opencv_interop.h>
#include <r4d_common/gtsam_opencv_interop.h>

namespace fiducial_slam {

    /// Robot localization using a single marker
    struct MultimarkerLocalizer {
        MultimarkerLocalizer(ros::NodeHandle &nh, ros::NodeHandle &nh_p) :
                nh_(nh),
                nh_p_(nh_p),
                pub_ic_(nh_.advertise<nav_msgs::Odometry>("/p_ic", 10)),
                pub_map_(nh_.advertise<rcars_detector_msgs::TagArray>("/markers_map", 10)),
                pub_extpose_(nh_.advertise<nav_msgs::Odometry>("/ext_pose", 10)),
                pub_origin_marker_(nh_.advertise<nav_msgs::Odometry>("/origin_marker", 10)),
                pub_solve_time_(nh_.advertise<std_msgs::Duration>("/solve_time", 10)),

                initialized_(false),
                pixel_noise_(gtsam::noiseModel::Isotropic::Sigma(8, 0.5)),
                landmark_noise_(gtsam::noiseModel::Isotropic::Sigma(6, 1e-6)),
                marker_prior_noise_(gtsam::noiseModel::Isotropic::Sigma(6, 1e-4)),
                motion_noise_(),
                tag_size_(0.16),
                motion_sigma_t_(1.0),
                motion_sigma_r_(1.0),
                ic_counter_(1) {

            nh_p_.param("motion_sigma_t", motion_sigma_t_, motion_sigma_t_);
            nh_p_.param("motion_sigma_r", motion_sigma_r_, motion_sigma_r_);
            ROS_WARN_STREAM("Using sigmas " << motion_sigma_r_ << " " << motion_sigma_t_);
            motion_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
                    (gtsam::Vector(6) <<
                            motion_sigma_r_,
                            motion_sigma_r_,
                            motion_sigma_r_,
                            motion_sigma_t_,
                            motion_sigma_t_,
                            motion_sigma_t_).finished());
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
                nav_msgs::Odometry odom;
                odom.header = header;
                odom.pose.pose = gtsam_pose_to_gmsgs(pMC);
                pub_extpose_.publish(odom);
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

        void tags_cb(const rcars_detector_msgs::TagArrayConstPtr msg) {
            // Symbol for the current camera pose
            gtsam::Symbol s_ic('x', ic_counter_);

            if (!K_) {
                ROS_WARN_THROTTLE(1, "Uncalibrated camera!");
                return;
            }

            if (!msg->tags.size()) {
                ROS_WARN_THROTTLE(1, "Skipping frame without markers.");
                return;
            }

            auto origin_it = std::find_if(msg->tags.begin(), msg->tags.end(), [&](const rcars_detector_msgs::Tag &tag) {
                return tag.id == 0;
            });
            if (origin_it == msg->tags.end()) {
                // return;
            }

            if (!initialized_) {
                initialized_ = true;

                // TODO: allow setting the marker used as origin
                auto &origin_tag = msg->tags[0];
                origin_id_ = origin_tag.id;

                ROS_INFO_STREAM("Using marker " << static_cast<int>(*origin_id_) << " as origin.");

                // Add a prior on the "origin" marker to set the origin
                gtsam::Symbol symb_origin('M', origin_tag.id);
                gtsam::PriorFactor<gtsam::Pose3> origin_prior(symb_origin,
                                                              gtsam::Pose3(),
                                                              landmark_noise_);
                graph_.push_back(origin_prior);
                estimate_.insert(symb_origin, gtsam::Pose3());

                // Add the first (and only) estimate on the camera pose
                // based on the detector's estimate
                auto guess = gmsgs_pose_to_gtsam(origin_tag.pose).inverse();
                estimate_.insert(s_ic, guess);
            }

            gtsam::Symbol s_prev_ic('x', ic_counter_ - 1);
            if (ic_counter_ > 0 && estimate_.exists(s_prev_ic)) {
                estimate_.insert(s_ic, estimate_.at(s_prev_ic));

                // Add motion prior (zero velocity in this case)
                graph_.push_back(gtsam::BetweenFactor<gtsam::Pose3>(s_prev_ic, s_ic, gtsam::Pose3(), motion_noise_));
            }

            std::cout << ic_counter_ << " ";
            for (auto const &tag : msg->tags) {
                // TODO: change me!
                if (tag.id != 0) {
                    // continue;
                }
                std::cout << static_cast<int>(tag.id) << " | ";
                gtsam::Symbol s_marker('M', tag.id);
                auto corners = tag_to_gtsam_points(tag);

                if (!estimate_.exists(s_marker)) {
                    // It's the first time we see this new marker: add an estimate
                    // based on the detector pose
                    auto init_pose = estimate_.at<gtsam::Pose3>(s_ic) * gmsgs_pose_to_gtsam(tag.pose);
                    // estimate_.insert(s_marker, init_pose);

                    gtsam::NonlinearFactorGraph g;
                    gtsam::Values v;
                    auto current_ic = estimate_.at<gtsam::Pose3>(s_ic);
                    g.push_back(gtsam::PriorFactor<gtsam::Pose3>(s_ic, current_ic, landmark_noise_));
                    v.insert(s_ic, current_ic);
                    g.push_back(MarkerFactor(corners, pixel_noise_, s_ic, s_marker, K_, tag_size_));
                    v.insert(s_marker, init_pose);
                    auto r = gtsam::GaussNewtonOptimizer(g, v).optimize();

                    auto bproj = r.at<gtsam::Pose3>(s_marker);
                    estimate_.insert(s_marker, bproj);

                    // graph_.push_back(
                    //         gtsam::PriorFactor<gtsam::Pose3>(s_marker, init_pose, marker_prior_noise_));

                    init_pose.print("From message was\n");
                }

                graph_.push_back(MarkerFactor(corners, pixel_noise_, s_ic, s_marker, K_, tag_size_));
            }
            std::cout << "\n";

            // estimate_ = gtsam::GaussNewtonOptimizer(graph_, estimate_).optimize();
            auto time_begin = ros::Time::now();
            estimate_ = gtsam::LevenbergMarquardtOptimizer(graph_, estimate_).optimize();
            auto time_end = ros::Time::now();

            // Publish time needed for solution
            {
                std_msgs::Duration duration_msg;
                duration_msg.data = time_end - time_begin;
                pub_solve_time_.publish(duration_msg);
            }

            auto gtsam_ic = estimate_.at<gtsam::Pose3>(s_ic);

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

            // Publish GTSAM TF and odometry
            {
                send_tf(
                    gtsam_pose_to_gmsgs(gtsam_ic.inverse()), "gtsam");

                pub_ic_.publish(
                    pose_to_odometry(
                        gtsam_pose_to_gmsgs(gtsam_ic.inverse())));

                if (!first_cam_pose_) {
                    first_cam_pose_ = gtsam_ic.inverse();
                }
            }

            // While the origin marker is visible, also publish the pose estimate by the detector
            {
                auto origin_it = std::find_if(msg->tags.begin(), msg->tags.end(), [&](const rcars_detector_msgs::Tag &tag) {
                    return tag.id == *origin_id_;
                });

                if (origin_it != msg->tags.end()) {
                    send_tf(origin_it->pose, "dtct");

                    pub_origin_marker_.publish(
                        pose_to_odometry(
                            origin_it->pose));
                }
            }

            // Publish the "map", i.e. the absolute position of all the markers we have ever seen
            {
                rcars_detector_msgs::TagArray map;
                map.header = msg->header;

                auto tag_filter = [](const gtsam::Key& k) {
                    return (k >= gtsam::Symbol('M', 0)) && (k < gtsam::Symbol('N', 0));
                };
                for (const auto& kv : estimate_.filter(tag_filter)) {
                    rcars_detector_msgs::Tag elem;
                    // The tag id was stored as the index in the node's gtsam::Symbol
                    gtsam::Symbol s(kv.key);
                    elem.id = s.index();
                    auto this_tag_pose = estimate_.at<gtsam::Pose3>(s);
                    elem.pose = gtsam_pose_to_gmsgs(this_tag_pose);
                    map.tags.push_back(elem);
                }
                pub_map_.publish(map);
            }

            if (ic_counter_ == 1) {
                std::ofstream os("/Users/niko/graph_2.dot");
                graph_.saveGraph(os);
            }

            ic_counter_ += 1;
        }

        void imu_cb(const sensor_msgs::ImuConstPtr &m) {

        }

        boost::shared_ptr<gtsam::Cal3DS2> K_;
        gtsam::NonlinearFactorGraph graph_;
        gtsam::Values estimate_;
        size_t ic_counter_; // Counter for the last camera pose id
        bool initialized_;
        boost::optional<unsigned char> origin_id_; // id of the marker used as origin
        boost::optional<gtsam::Pose3> first_cam_pose_;
        boost::optional<gtsam::Pose3> ground_truth_calib_;

        gtsam::noiseModel::Isotropic::shared_ptr pixel_noise_;
        gtsam::noiseModel::Isotropic::shared_ptr landmark_noise_;
        gtsam::noiseModel::Isotropic::shared_ptr marker_prior_noise_;
        gtsam::noiseModel::Diagonal::shared_ptr motion_noise_;

        double tag_size_;
        double motion_sigma_t_, motion_sigma_r_; // Sigma for the motion noise model

        ros::NodeHandle nh_, nh_p_;
        tf::TransformBroadcaster tf_br_;
        ros::Publisher pub_ic_, pub_map_, pub_extpose_, pub_origin_marker_, pub_solve_time_;
    };
}