/**
 * @file MarkerFactor.h
 * @brief Factors for square fiducial marker observation
 */

#pragma once

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/SimpleCamera.h>

namespace fiducial_slam {
    using MARKER_POSE = gtsam::Pose3;

    /*
     * Factor for a constraint derived when observing a square fiducial marker.
     */
    class MarkerFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, MARKER_POSE> {
        std::array<gtsam::Point2, 4> corners_;
        boost::shared_ptr<gtsam::Cal3DS2> K_;
        double half_tag_size_;
        gtsam::Pose3 body_P_sensor_;

    public:
        typedef gtsam::NoiseModelFactor2<gtsam::Pose3, MARKER_POSE> Base;
        typedef fiducial_slam::MarkerFactor This;
        typedef boost::shared_ptr<This> shared_ptr;

        /*
         * Constructor
         * @param corners are the image positions of the corner of the observed marker
         * @param model is the noise model
         * @param K shared pointer to the constant camera calibration
         * @param tag_size edge length of the marker
         */
        MarkerFactor(const std::array<gtsam::Point2, 4> &corners,
                     const gtsam::SharedNoiseModel &model,
                     gtsam::Key cam_key,
                     gtsam::Key marker_key,
                     const boost::shared_ptr<gtsam::Cal3DS2> &K,
                     double tag_size,
                     gtsam::Pose3 body_P_sensor = gtsam::Pose3()) :
                Base(model, cam_key, marker_key),
                corners_(corners),
                K_(K),
                half_tag_size_(0.5 * tag_size),
                body_P_sensor_(body_P_sensor) { }

        virtual ~MarkerFactor() { }

        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new This(*this))
            );
        }

        /// Evaluate error h(x)-z and optionally derivatives
        gtsam::Vector evaluateError(const gtsam::Pose3 &body_pose,
                                    const MARKER_POSE &marker_pose,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none,
                                    boost::optional<gtsam::Matrix &> H2 = boost::none) const {
            // Jacobian d(camera in world)/d(body in world)
            gtsam::Matrix Hbody;
            auto cam_pose = body_pose.compose(body_P_sensor_, Hbody);

            gtsam::PinholeCamera<gtsam::Cal3DS2> camera (cam_pose, *K_);

            // Jacobians have one row for each component of the residual vector
            if (H1) {
                *H1 = Eigen::Matrix<double, 8, 6>::Zero();
            }
            if (H2) {
                *H2 = Eigen::Matrix<double, 8, 6>::Zero();
            }

            gtsam::Matrix Hcorner, Hcamera;

            gtsam::Matrix residuals;
            residuals = Eigen::Matrix<double, 8, 1>::Ones() * 2.0 * K_->fx();

            for (int i = 0; i < 4; i++) {
                // Jacobian d(point in world) / d(marker pose)
                gtsam::Matrix H0;
                // Coordinates of the current corner in the marker frame
                gtsam::Point3 marker_corner(i % 2 == 1 ? half_tag_size_ : -half_tag_size_,
                                            i < 2 ? half_tag_size_ : -half_tag_size_,
                                            0);
                // Use the marker pose to transform into world coordinates
                auto world_corner = marker_pose.transform_from(marker_corner, H0);

                try {
                    gtsam::Point2 reproj_error(
                            camera.project(world_corner, Hcamera, Hcorner, boost::none) - corners_[i]);

                    // Jacobian w.r.t. body ( dres/dcam * dcam/dbody)
                    if (H1) {
                        (*H1).block<2, 6>(2 * i, 0) = Hcamera * Hbody;
                    }

                    // Jacobian w.r.t. marker pose
                    if (H2) {
                        // Chain Jacobians of marker to corner transformation and camera projection
                        (*H2).block<2, 6>(2 * i, 0) = Hcorner * H0;
                    }

                    // Copy in the right block of the complete residual vector
                    residuals.block<2, 1>(2 * i, 0) = reproj_error.vector();
                } catch (gtsam::CheiralityException &e) {
                    // One of the corners has moved behind the camera
                    std::cout << "CExc\n";
                }
            }

            return residuals;
        }
    };
}