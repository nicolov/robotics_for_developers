/*
 * @file BagRunner.h
 * @brief Templated helper class to load messages from bags
 */

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <rcars_detector_msgs/TagArray.h>

namespace fiducial_slam {

    template <class Localizer>
    struct BagRunner {
        BagRunner (ros::NodeHandle& nh, ros::NodeHandle& nh_p) : 
                                          nh_(nh),
                                          nh_p_(nh_p),
                                          localizer_(nh_, nh_p_) {
            std::string bag_fname("/Users/niko/rcars_table_markers.bag");
            nh_p_.param("bag_fname", bag_fname, bag_fname);

            ROS_WARN_STREAM("Opening bag " << bag_fname);
            bag_.open(bag_fname, rosbag::bagmode::Read);
        }

        void run() {
            rosbag::View view(bag_);
            ros::Rate r(10);

            for (auto const &msg : view) {
                if (msg.getTopic() == "/cam0/camera_info") {
                    auto caminfo_msg = msg.instantiate<sensor_msgs::CameraInfo>();
                    localizer_.image_info_cb(caminfo_msg);
                } else if (msg.getTopic() == "/rcars/detector/tags") {
                    auto tags_msg = msg.instantiate<rcars_detector_msgs::TagArray>();
                    localizer_.tags_cb(tags_msg);
                } else if (msg.getTopic() == "/vicon/auk/auk") {
                    auto gtruth_msg = msg.instantiate<geometry_msgs::TransformStamped>();
                    localizer_.ground_truth_cb(gtruth_msg);
                } else if (msg.getTopic() == "/rcars/estimator/filterPose") {
                    auto odom_msg = msg.instantiate<nav_msgs::Odometry>();
                    localizer_.filter_output_cb(odom_msg);
                } else if (msg.getTopic() == "/imu0") {
                    auto imu_msg = msg.instantiate<sensor_msgs::Imu>();
                    localizer_.imu_cb(imu_msg);
                }

                ros::spinOnce();
                if (!ros::ok()) {
                    break;
                }
            }

            bag_.close();
        }

        ros::NodeHandle nh_, nh_p_;
        Localizer localizer_;
        rosbag::Bag bag_;
    };

}