/*
 * @file single_marker_test.cpp
 * @brief Bag loading code for the SingleMarkerLocalizer class
 */

#include <ros/ros.h>

#include <r4d_common/BagRunner.h>
#include <r4d_1_single_marker/SingleMarkerLocalizer.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "single_marker_test");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    fiducial_slam::BagRunner<fiducial_slam::SingleMarkerLocalizer> runner(nh, nh_p);
    runner.run();
}