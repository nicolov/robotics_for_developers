/*
 * @file multi_marker_test.cpp
 * @brief Bag loading code for the ImuLocalizer class
 */

#include <ros/ros.h>

#include <r4d_common/BagRunner.h>
#include <r4d_3_imu/ImuLocalizer.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "multi_marker_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    fiducial_slam::BagRunner<fiducial_slam::ImuLocalizer> runner(nh, nh_p);
    runner.run();
}