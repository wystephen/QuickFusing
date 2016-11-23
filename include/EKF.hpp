#pragma once
//
// Created by steve on 16-11-23.
//

//Create by steve in 16-11-23 at 下午8:02

#include "SettingPara.h"


#ifndef QUICKFUSING_EKF_HPP
#define QUICKFUSING_EKF_HPP


class Ekf {
public:
    Ekf(SettingPara para) {

        para_ = para;

    }

    bool InitNavEq(Eigen::MatrixXd u) {

        double f_u(0.0), f_v(0.0), f_w(0.0);

        f_u = u.col(0).mean();
        f_v = u.col(1).mean();
        f_w = u.col(2).mean();

        std::cout << "u,v,w:" << f_u << "," << f_v << "," << f_w << std::endl;


        double roll(atan2(-f_v, -f_w)), pitch(atan2(f_u, sqrt(f_v * f_v + f_w * f_w)));

        Eigen::Vector3d attitude(roll, pitch, para_.init_heading1_);


        return true;
    }


protected:

    /*
     * Euler to Rotation Matrix.
     */
    Eigen::MatrixXd Rt2b(Eigen::Vector3d ang) {
        double cr(cos(ang[0])), sr(sin(ang[0]));
        double cp(cos(ang[1])), sp(sin(ang[1]));
        double cy(cos(ang[2])), sy(sin(ang[2]));

        Eigen::Matrix3d R;

        R(0, 0) = cy * cp;
        R(0, 1) = sy * cp;
        R(0, 2) = -sp;

        R(1, 0) = -sy * cr + cy * sp * sr;
        R(1, 1) = cy * cr + sy * sp * sr;
        R(1, 2) = cp * sr;

        R(2, 0) = sy * sr + cy * sp * cr;
        R(2, 1) = -cy * sr + sy * sp * cr;
        R(2, 2) = cp * cr;

        return R;
    }


private:
    //Parameters in here.
    SettingPara para_;

    //P for single foot
    Eigen::Matrix<double, 9, 9> P_;

    Eigen::Matrix<double, 6, 6> Q_;

    Eigen::Matrix<double, 3, 9> H_;

    Eigen::Matrix<double, 9, 1> x_h_;






};


#endif //QUICKFUSING_EKF_HPP
