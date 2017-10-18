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

class MagConstraintFactor :public gtsam::NoiseModelFactor2<gtsam::Pose3,gtsam::Point3>{

    /**
     *  find local magnetic field at http://www.ngdc.noaa.gov/geomag-web/#igrfwmm
     */
    const gtsam::Point3 measured_; /// < the measured magnetomeer values
    const gtsam::Point3 nM_; /// < local magnetic field (mag output units)
//    const gtsam::Point3 bias_; /// < bias

public:
    MagConstraintFactor(gtsam::Key key_pose,
    gtsam::Key key_bias,
    const gtsam::Point3& measured,
    const gtsam::Point3& nM,
    const gtsam::SharedNoiseModel& model):
            gtsam::NoiseModelFactor2<gtsam::Pose3,gtsam::Point3>(model,key_pose,key_bias),
            measured_(measured),nM_(nM){

    }





};

#endif //QUICKFUSING_MAGCONSTRAINTFACTOR_H
