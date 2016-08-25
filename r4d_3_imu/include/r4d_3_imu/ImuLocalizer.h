/*
 * @file ImuLocalizer.h
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
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>

#include <r4d_common/MarkerFactor.h>
#include <r4d_common/ros_gtsam_interop.h>
#include <r4d_common/ros_opencv_interop.h>
#include <r4d_common/gtsam_opencv_interop.h>

namespace fiducial_slam {
    using gtsam::symbol_shorthand::X; // Body pose
    using gtsam::symbol_shorthand::V; // Velocity
    using gtsam::symbol_shorthand::B; // Biases
    using gtsam::symbol_shorthand::M; // Marker pose

    /// Robot localization using a single marker
    struct ImuLocalizer {
        ImuLocalizer(ros::NodeHandle &nh, ros::NodeHandle &nh_p) :
                nh_(nh),
                nh_p_(nh_p),
                pub_ic_(nh_.advertise<nav_msgs::Odometry>("/p_ic", 10)),
                pub_map_(nh_.advertise<rcars_detector_msgs::TagArray>("/markers_map", 10)),
                pub_extpose_(nh_.advertise<nav_msgs::Odometry>("/ext_pose", 10)),
                pub_origin_marker_(nh_.advertise<nav_msgs::Odometry>("/origin_marker", 10)),
                pub_biases_(nh_.advertise<sensor_msgs::Imu>("/imu_biases", 10)),
                pub_prediction_(nh_.advertise<nav_msgs::Odometry>("/p_ic_pred", 10)),

                landmark_noise_(gtsam::noiseModel::Isotropic::Sigma(6, 1e-6)),
                vel_init_noise_(gtsam::noiseModel::Isotropic::Sigma(3, 1e-2)),
                tag_size_(0.16),
                accel_init_done_(false),
                pose_init_done_(false),
                gravity_magn_(9.8),
                ic_counter_(1) {

            nh_p.param<double>("accel_sigma", accel_sigma_, 1e-6);
            nh_p.param<double>("gyro_sigma", gyro_sigma_, 4e-6);
            nh_p.param<double>("accel_bias_sigma", accel_bias_sigma_, 1e-3);
            nh_p.param<double>("gyro_bias_sigma", gyro_bias_sigma_, 1e-3);
            nh_p.param<double>("bias_prior_sigma", bias_prior_sigma_, 1e-3);
            nh_p.param<double>("pixel_sigma", pixel_sigma_, 0.5);

            ROS_WARN_STREAM(
                    "\naccel_sigma = " << accel_sigma_ <<
                    "\ngyro_sigma = " << gyro_sigma_ <<
                    "\naccel_bias_sigma = " << accel_bias_sigma_ <<
                    "\ngyro_bias_sigma = " << gyro_bias_sigma_ <<
                    "\nbias_prior_sigma = " << bias_prior_sigma_ <<
                    "\npixel_sigma = " << pixel_sigma_);

            // If/when to pause marker-based correction for evaluation
            {
                int p;
                nh_p.param<int>("pause_correction_start", p, 0);
                if (p > 0) {
                    pause_correction_start_ = p;
                    nh_p.param<int>("pause_correction_duration", pause_correction_duration_, 0);
                    ROS_WARN_STREAM(
                            "Eval: Disabling correction at frame " << p << " for " << pause_correction_duration_ <<
                            " frames");
                }
            }

            nh_p.param<int>("correction_skip", correction_skip_, 1);

            /*
            vea_x 0.0083632
            vea_y -0.00390811
            vea_z 0.712747
            vea_w 0.70136
            vep_0 -0.0116603
            vep_1 -0.0570516
            vep_2 -0.00124568
             */

            extrinsics_ = gtsam::Pose3(gtsam::Rot3::Quaternion(0.7, 0, 0, 0.7), gtsam::Point3(0.05, 0, 0));
            bias_prior_noise_ = gtsam::noiseModel::Isotropic::Sigma(6, bias_prior_sigma_);
            pixel_noise_ = gtsam::noiseModel::Isotropic::Sigma(8, pixel_sigma_);

            preint_params_ = gtsam::PreintegrationParams::MakeSharedU(gravity_magn_);
            preint_params_->accelerometerCovariance = accel_sigma_ * gtsam::I_3x3;
            preint_params_->gyroscopeCovariance = gyro_sigma_ * gtsam::I_3x3;
            preint_params_->integrationCovariance = 1e-8 * gtsam::I_3x3;

            bias_between_sigmas_vec_ = (gtsam::Vector(6) << gtsam::Vector3::Constant(accel_bias_sigma_),
                    gtsam::Vector3::Constant(gyro_bias_sigma_)).finished();
        }

        void image_info_cb(const sensor_msgs::CameraInfoConstPtr &m) {
            if (!K_) {
                K_ = fiducial_slam::caminfo_to_gtsam_calib(m);
                K_->print("Calibration from camera_info msg:\n");
            }
        }

        void handle_external_pose(const geometry_msgs::Pose &ext_pose, const std_msgs::Header &header,
                                  std::string label) {
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

        void tags_cb(const rcars_detector_msgs::TagArrayConstPtr &msg) {
            // Symbol for the current camera pose
            auto s_ic = X(ic_counter_);
            auto s_vel = V(ic_counter_);
            auto s_bias = B(ic_counter_);

            if (!K_) {
                ROS_WARN_THROTTLE(1, "Uncalibrated camera!");
                return;
            }

            if (!accel_init_done_) {
                ROS_WARN_STREAM(
                        "Received markers, but ignoring them until IMU data is available to initialize");
                return;
            }

            // While the origin marker is visible, also publish the pose estimate by the detector
            // We do it here so it's available even when artificially dropping markers
            if (origin_id_) {
                publish_marker_by_id(*origin_id_, msg);
            }

            // For eval purposes, disable corrections and within a certain window
            if (pause_correction_start_ && ic_counter_ == *pause_correction_start_ && pause_correction_duration_ > 0) {
                ROS_WARN_STREAM("Skipping frame (" << pause_correction_duration_ << ") remaining");
                pause_correction_duration_ -= 1;
                return;
            }

            if (!pose_init_done_) {
                pose_init_done_ = true;

                // Add estimate and prior for initial body pose
                graph_.push_back(
                        gtsam::PriorFactor<gtsam::Pose3>(s_ic, initial_pose_, landmark_noise_));
                estimate_.insert(s_ic, initial_pose_);

                // Add an estimate and prior for zero initial velocity
                graph_.push_back(
                        gtsam::PriorFactor<gtsam::Vector3>(s_vel, gtsam::Vector3(), vel_init_noise_));
                estimate_.insert(s_vel, gtsam::Vector3());

                // Add an estimate and prior for zero initial bias
                graph_.push_back(
                        gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
                                s_bias, gtsam::imuBias::ConstantBias(), bias_prior_noise_));
                estimate_.insert(s_bias, gtsam::imuBias::ConstantBias());
            }

            auto s_prev_ic = X(ic_counter_ - 1);
            auto s_prev_vel = V(ic_counter_ - 1);
            auto s_prev_bias = B(ic_counter_ - 1);
            if (ic_counter_ > 0 && estimate_.exists(s_prev_ic)) {
                // TODO: use the imu data to predict the new pose/velocity
                estimate_.insert(s_ic, estimate_.at(s_prev_ic));
                estimate_.insert(s_vel, estimate_.at(s_prev_vel));
                estimate_.insert(s_bias, estimate_.at(s_prev_bias));

                auto prev_bias = estimate_.at<gtsam::imuBias::ConstantBias>(s_prev_bias);
                gtsam::PreintegratedImuMeasurements pre_integr_data(preint_params_, prev_bias);

                int num_imus_found = 0;
                for (const auto &kv : imu_meas_) {
                    if (kv.first > last_image_stamp_ && kv.first <= msg->header.stamp) {
                        num_imus_found += 1;
                        pre_integr_data.integrateMeasurement(kv.second.first, kv.second.second, 0.005);
                    }
                }

                // Output IMU prediction
                {
                    gtsam::NavState nav_begin(estimate_.at<gtsam::Pose3>(s_prev_ic),
                                              estimate_.at<gtsam::Vector3>(s_prev_vel));
                    auto nav_predicted = pre_integr_data.predict(nav_begin, estimate_.at<gtsam::imuBias::ConstantBias>(
                            s_prev_bias));
                    auto new_pose_ros = gtsam_pose_to_gmsgs(nav_predicted.pose().inverse());

                    nav_msgs::Odometry odom;
                    odom.header.stamp = msg->header.stamp;
                    odom.pose.pose = new_pose_ros;
                    pub_prediction_.publish(odom);
                }
                last_image_stamp_ = msg->header.stamp;

                // IMU factor
                graph_.add(
                        gtsam::ImuFactor(s_prev_ic, s_prev_vel, s_ic, s_vel, s_bias, pre_integr_data));
                // Bias evolution factor
                auto bias_between_noise = gtsam::noiseModel::Diagonal::Sigmas(
                        sqrt(pre_integr_data.deltaTij()) * bias_between_sigmas_vec_);
                graph_.add(
                        gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(s_prev_bias,
                                                                           s_bias,
                                                                           gtsam::imuBias::ConstantBias(),
                                                                           bias_between_noise));
            }

            bool use_this_correction = (ic_counter_-1) % correction_skip_ == 0;

            std::cout << ic_counter_ << ": ";
            for (auto const &tag : msg->tags) {
                gtsam::Symbol s_marker('M', tag.id);

                std::cout << static_cast<int>(tag.id) << " | ";

                if (!estimate_.exists(s_marker)) {
                    // It's the first time we see this new marker: add an estimate
                    // based on the detector pose
                    auto detector_pose = gmsgs_pose_to_gtsam(tag.pose);
                    auto init_pose = estimate_.at<gtsam::Pose3>(s_ic) * detector_pose;
                    estimate_.insert(s_marker, init_pose);

                    if (!origin_id_) {
                        // This tag will also be used as reference when comparing to the detector output
                        origin_id_ = tag.id;
                        ROS_INFO_STREAM("Using marker " << static_cast<int>(*origin_id_) << " as origin.");
                        first_origin_marker_pose_ = detector_pose;
                    }
                }

                if (use_this_correction) {
                    auto corners = tag_to_gtsam_points(tag);
                    graph_.push_back(
                            MarkerFactor(corners, pixel_noise_, s_ic, s_marker, K_, tag_size_, extrinsics_));
                }
            }
            if (!use_this_correction) {
                std::cout << "(s)";
            }

            std::cout << "\n";

            estimate_ = gtsam::GaussNewtonOptimizer(graph_, estimate_).optimize();

            if (!first_cam_pose_) {
                auto gtsam_ic = estimate_.at<gtsam::Pose3>(s_ic);
                first_cam_pose_ = gtsam_ic.inverse();
            }

            publish_gtsam_navstate(estimate_, msg->header, ic_counter_);
            publish_map(estimate_, msg->header);

            ic_counter_ += 1;
        }

        void do_accel_init() {
            gtsam::Vector3 acc_avg;
            for (auto &kv : imu_meas_) {
                acc_avg += kv.second.first;
            }
            acc_avg /= imu_meas_.size();
            ROS_WARN_STREAM("Gravity-aligning with accel. vector:\n" << acc_avg);

            gtsam::Vector3 gravity_vec;
            gravity_vec << 0.0, 0.0, gravity_magn_;
            auto initial_att = gtsam::Rot3(Eigen::Quaterniond().setFromTwoVectors(acc_avg, gravity_vec));
            initial_pose_ = gtsam::Pose3(initial_att, gtsam::Point3());

            (initial_pose_ * acc_avg).print("Gravity vector after alignment:\n");

            accel_init_done_ = true;
        }

        void imu_cb(const sensor_msgs::ImuConstPtr &m) {
            auto acc = gtsam::Vector3(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z);
            auto gyro = gtsam::Vector3(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);
            imu_meas_[m->header.stamp] = std::make_pair(acc, gyro);

            // Wait for 10 accelerometer measurements before aligning to gravity
            if (!accel_init_done_ && imu_meas_.size() > 10) {
                do_accel_init();
            }
        }

        void publish_gtsam_navstate(const gtsam::Values &values,
                                    const std_msgs::Header &header,
                                    size_t step) {
            auto gtsam_ic = values.at<gtsam::Pose3>(X(step));
            auto gtsam_vel = values.at<gtsam::Vector3>(V(step));
            auto gtsam_bias = values.at<gtsam::imuBias::ConstantBias>(B(step));

            tf::Transform tf_transf;
            tf::poseMsgToTF(gtsam_pose_to_gmsgs(gtsam_ic.inverse()), tf_transf);
            tf_br_.sendTransform(tf::StampedTransform(tf_transf, header.stamp, "world", "gtsam"));

            nav_msgs::Odometry odom;
            odom.header.stamp = header.stamp;
            odom.pose.pose = gtsam_pose_to_gmsgs(gtsam_ic.inverse());
            odom.twist.twist.linear = gtsam_vector_to_gmsgs(gtsam_vel);
            pub_ic_.publish(odom);

            sensor_msgs::Imu imu;
            imu.header.stamp = header.stamp;
            imu.linear_acceleration.x = gtsam_bias.accelerometer()(0);
            imu.linear_acceleration.y = gtsam_bias.accelerometer()(1);
            imu.linear_acceleration.z = gtsam_bias.accelerometer()(2);
            imu.angular_velocity.x = gtsam_bias.gyroscope()(0);
            imu.angular_velocity.y = gtsam_bias.gyroscope()(1);
            imu.angular_velocity.z = gtsam_bias.gyroscope()(2);
            pub_biases_.publish(imu);
        }

        void publish_marker_by_id(unsigned char id, const rcars_detector_msgs::TagArrayConstPtr &msg) {
            auto origin_it = std::find_if(msg->tags.begin(), msg->tags.end(),
                                          [&](const rcars_detector_msgs::Tag &tag) {
                                              return tag.id == *origin_id_;
                                          });
            if (origin_it != msg->tags.end()) {
                tf::Transform tf_transf;
                // Compose the detector pose with the initial origin marker pose and extrinsics
                // to align with gtsam output
                auto detector_pose = gmsgs_pose_to_gtsam(origin_it->pose);
                // Without extrinsics:
                // auto new_pose = detector_pose * first_origin_marker_pose_.inverse();
                // With extrinsics and initial gravity alignment:
                // WB1 = WB0 * BC * C0M * (C1M)^-1 * (BC)^-1
                auto new_pose =
                        extrinsics_ * detector_pose * first_origin_marker_pose_.inverse() * extrinsics_.inverse() *
                        initial_pose_.inverse();
                auto new_pose_ros = gtsam_pose_to_gmsgs(new_pose);

                tf::poseMsgToTF(new_pose_ros, tf_transf);
                tf_br_.sendTransform(tf::StampedTransform(tf_transf, msg->header.stamp, "world", "dtct"));

                nav_msgs::Odometry odom;
                odom.header.stamp = msg->header.stamp;
                odom.pose.pose = new_pose_ros;
                pub_origin_marker_.publish(odom);
            }
        }

        void publish_map(const gtsam::Values &values, const std_msgs::Header &header) {
            // Publish the "map", i.e. the absolute position of all the markers we have ever seen
            rcars_detector_msgs::TagArray map;
            map.header = header;

            auto tag_filter = [](const gtsam::Key &k) {
                return (k >= gtsam::Symbol('M', 0)) && (k < gtsam::Symbol('N', 0));
            };
            for (const auto &kv : estimate_.filter(tag_filter)) {
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

        boost::shared_ptr<gtsam::Cal3DS2> K_;
        gtsam::NonlinearFactorGraph graph_;
        gtsam::Values estimate_;

        // Coordinate frames stuff
        size_t ic_counter_; // Counter for the last camera pose id
        bool pose_init_done_;
        boost::optional<unsigned char> origin_id_; // id of the marker used as reference
        gtsam::Pose3 first_origin_marker_pose_;
        boost::optional<gtsam::Pose3> first_cam_pose_;
        boost::optional<gtsam::Pose3> ground_truth_calib_;
        bool accel_init_done_;
        gtsam::Pose3 initial_pose_; // Gets reset by IMU measurements to be 0, 0, 0 and gravity-aligned

        // IMU stuff
        double gravity_magn_;
        boost::shared_ptr<gtsam::PreintegrationParams> preint_params_;
        double accel_sigma_, gyro_sigma_;
        double accel_bias_sigma_, gyro_bias_sigma_;
        gtsam::Pose3 extrinsics_; // Camera <-> IMU extrinsics
        gtsam::Vector6 bias_between_sigmas_vec_; // Vector of 6 sigmas for bias evolution

        // Measurements stuff
        std::map<ros::Time, std::pair<gtsam::Vector3, gtsam::Vector3>> imu_meas_;
        ros::Time last_image_stamp_;
        double tag_size_;

        // Noise models
        double bias_prior_sigma_, pixel_sigma_;
        gtsam::noiseModel::Isotropic::shared_ptr pixel_noise_, landmark_noise_, vel_init_noise_, vel_integration_noise_;
        gtsam::noiseModel::Diagonal::shared_ptr bias_prior_noise_;

        // Evaluation stuff
        boost::optional<int> pause_correction_start_;
        int pause_correction_duration_;
        int correction_skip_;

        ros::NodeHandle nh_, nh_p_;
        tf::TransformBroadcaster tf_br_;
        ros::Publisher pub_ic_, pub_map_, pub_extpose_, pub_origin_marker_, pub_biases_, pub_prediction_;
    };
}
