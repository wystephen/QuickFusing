//
// Created by steve on 16-9-11.
//

#include <deque>
#include <iostream>

#include "SettingPara.h"

#include <memory>

#include <Eigen/Dense>

#ifndef IMU_NAVIGATION_ZERO_DETECTER_H
#define IMU_NAVIGATION_ZERO_DETECTER_H
bool GLRT_Detector(Eigen::MatrixXd u,
                   const SettingPara &para_) {
    Eigen::Vector3d ya_m;
    double g = para_.gravity_;

    double T(0.0);
//    Eigen::MatrixXd Tmatrix(1, 1);
//    Tmatrix

    for (int i(0); i < 3; ++i) {
        ya_m(i) = u.block(i, 0, 1, u.cols()).mean();
    }

    Eigen::Vector3d tmp;

    for (int i(0); i < u.cols(); ++i) {

        tmp = u.block(0, i, 3, 1) - g * ya_m / ya_m.norm();
        if (std::isnan(tmp.sum())) {
            std::cout << "nan at tmp in " << __FUNCTION__ << ":"
                      << __FILE__ << ":" << __LINE__ << std::endl;
        }

//        std::cout << " u block size : " << u.block(3,i,3,1).rows()<< std::endl;
//        std::cout << "tmp size :" << tmp.rows()<< std::endl;

        T += (u.block(3, i, 3, 1).transpose() * u.block(3, i, 3, 1) / para_.sigma_g_ +
              tmp.transpose() * tmp / para_.sigma_a_).sum();

//        if(std::isnan(Tmatrix.sum()))
//        {
//            std::cout << "Tmatrix is nan" << __FILE__
//                                          << ":"<< __LINE__ << std::endl;
//        }


    }

//    if (Tmatrix.size() != 1) {
//        MYERROR("Tmatrxi size is not equal to 1")
//    }


    T = T / double(para_.ZeroDetectorWindowSize_);

//    std::cout << "T :" << T << std::endl;
    if (T < para_.gamma_) {
        return true;

    } else {

        return false;
    }

}

#endif //IMU_NAVIGATION_ZERO_DETECTER_H
