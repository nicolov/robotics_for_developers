/*
 * @file multi_marker_test.cpp
 * @brief Bag loading code for the MultimarkerLocalizer class
 */

#include <ros/ros.h>

#include <r4d_common/BagRunner.h>
#include <r4d_2_multimarker/MultimarkerLocalizer.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "multi_marker_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    fiducial_slam::BagRunner<fiducial_slam::MultimarkerLocalizer> runner(nh, nh_p);
    runner.run();
}