/** 
                   _ooOoo_ 
                  o8888888o 
                  88" . "88 
                  (| -_- |) 
                  O\  =  /O 
               ____/`---'\____ 
             .'  \\|     |//  `. 
            /  \\|||  :  |||//  \ 
           /  _||||| -:- |||||-  \ 
           |   | \\\  -  /// |   | 
           | \_|  ''\---/''  |   | 
           \  .-\__  `-`  ___/-. / 
         ___`. .'  /--.--\  `. . __ 
      ."" '<  `.___\_<|>_/___.'  >'"". 
     | | :  `- \`.;`\ _ /`;.`/ - ` : | | 
     \  \ `-.   \_ __\ /__ _/   .-` /  / 
======`-.____`-.___\_____/___.-`____.-'====== 
                   `=---=' 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
         佛祖保佑       永无BUG 
*/
//
// Created by steve on 17-10-14.
//

#ifndef QUICKFUSING_MAGCONSTRAINTFACTOR_H
#define QUICKFUSING_MAGCONSTRAINTFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>

#include <gtsam/geometry/Pose3.h>

namespace gtsam {
    class MagConstrainPoseFactor : public NoiseModelFactor1<Pose3> {

        const Vector3 measured_; ///< The measured magnetometer values
        const Vector3 nM_; ///< Local magnetic field (mag output units)
        const Vector3 bias_; ///< bias

    public:

        /** Constructor */
        MagConstrainPoseFactor(Key key,
                               const Point3 &measured,
                               double scale,
                               const Vector3 &direction,
                               const Vector3 &bias,
                               const SharedNoiseModel &model) :
                NoiseModelFactor1<Pose3>(model, key), //
                measured_(measured), nM_(scale * direction), bias_(bias) {
        }

        /// @return a deep copy of this factor
        virtual NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<NonlinearFactor>(
                    NonlinearFactor::shared_ptr(new MagConstrainPoseFactor(*this)));
        }

        /**
         * @brief vector of errors
         */
        Vector evaluateError(const Pose3 &nPb,
                             boost::optional<Matrix &> H = boost::none) const {
            // measured bM = nRb� * nM + b
            Vector3 hx = nPb.rotation().unrotate(nM_, H, boost::none) + bias_;
//            std::cout << (hx-measured_).transpose() << std::endl;
            return (hx - measured_);
//            return Vector3(0,0,0);
        }
    };
}


class MagConstraintFactor :
        public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {

    /**
     *  find local magnetic field at http://www.ngdc.noaa.gov/geomag-web/#igrfwmm
     *
     *  ############################################################
        #              Magnetic Field Components
        ############################################################
        #   8 Fields
        #     (1) Date in decimal years
        #     (2) Declination in decimal degrees
        #     (3) Inclination in decimal degrees
        #     (4) Horintensity in nanoTesla (nT)
        #     (5) Totalintensity in nanoTesla (nT)
        #     (6) Xcomponent in nanoTesla (nT)
        #     (7) Ycomponent in nanoTesla (nT)
        #     (8) Zcomponent in nanoTesla (nT)
        # NOTE: The first row is change per year in degrees or nanoTesla (nT);
        # NOTE: The second row is uncertainty in degrees or nanoTesla (nT):
        #
        #   Magnetic Model: WMM2015 (calculator version 0.5.0.7)
        #   Elevation: 0.00000 km Mean Sea Level
        #   Latitude: 39.90611 degrees, Longitude: 116.38806 degrees
        ############################################################
        change/year,-0.06105,0.07734,-46.7,32.2,-49.9,-24.1,65.5
        uncertainty,0.31,0.22,133,152,89,138,165
        2017.80274,-6.84582,59.09488,28049.0,54610.6,27849.0,-3343.4,46856.9
     */
    const gtsam::Point3 measured_; /// < the measured magnetomeer values
    const gtsam::Point3 nM_; /// < local magnetic field (mag output units)
//    const gtsam::Point3 bias_; /// < bias

public:
    /**
     * measured = Pose3.Rot3.rotation(nM) + bias
     * @param key_pose  a pose3,
     * @param key_bias  bias
     * @param measured
     * @param nM
     * @param model
     */
    MagConstraintFactor(gtsam::Key key_pose,
                        gtsam::Key key_bias,
                        const gtsam::Point3 &measured,
                        const gtsam::Point3 &nM,
                        const gtsam::SharedNoiseModel &model) :
            gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>(model, key_pose, key_bias),
            measured_(measured), nM_(nM) {


    }

    /**
     *
     * @return  deep copy of this factor
     */
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new MagConstraintFactor(*this))
        );

    }

    /**
     *
     * @brief vector of errors(vec 3)
     * @param pose
     * @param bias
     * @param H
     * @return
     */
    gtsam::Vector evaluateError(const gtsam::Pose3 &Pose,
                                const gtsam::Point3 &bias,
                                boost::optional<gtsam::Matrix &> H1 = boost::none,
                                boost::optional<gtsam::Matrix &> H2 = boost::none) const {
        gtsam::Point3 rotated_M = Pose.rotation().rotate(nM_, boost::none, H1) + bias;
        if (H2)
            *H2 = gtsam::I_3x3;

        return (rotated_M - measured_);


    }


};

#endif //QUICKFUSING_MAGCONSTRAINTFACTOR_H
