#pragma once
//
// Created by steve on 17-1-14.
//

#ifndef QUICKFUSING_IMUINTEGRATE_H
#define QUICKFUSING_IMUINTEGRATE_H

//#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <thread>
#include <MyError.h>
#include <time_stamp.h>


#include "sophus/so3.h"
#include "sophus/se3.h"

class ImuIntegrate {
public:
    ImuIntegrate(double error) {
        state_.resize(9);
        state_.setZero();
        time_step_ = 1e-10;
        tb2w = Eigen::Affine3d::Identity();
    }

    /**
     * Simple integration.
     * @param u
     * @param t
     * @return
     */
    Eigen::VectorXd IntegratingState(Eigen::VectorXd u, double t) {
        time_step_ = t;
        Eigen::VectorXd out;
        out.resize(6);

        Eigen::Affine3d T;

        Eigen::Matrix3d R;
        Eigen::Vector3d t_v;


        Eigen::VectorXd wv = u.block(3, 0, 3, 1);
        Eigen::VectorXd a = u.block(0, 0, 3, 1);


        std::cout << "a is: " << a.transpose() << std::endl;
        std::cout << "w v is : " << wv.transpose() << std::endl;


        Eigen::Matrix3d rotation_matrix;
        for (int i(0); i < 3; ++i) {
            for (int j(0); j < 3; ++j) {
                rotation_matrix(i, j) = tb2w(i, j);
            }
        }


        Eigen::VectorXd aworld = rotation_matrix * a;

        Sophus::SO3 SO3_R((rotation_matrix));

        Sophus::SO3 SO3_increase(wv(0) * t, wv(1) * t, wv(2) * t);

//        SO3_R =  Sophus::SO3::exp(wv*t) * SO3_R ;
//        SO3_R = SO3_R * Sophus::SO3::exp(wv*t);

//        SO3_R = SO3_R * SO3_increase;
//        SO3_R = SO3_increase.inverse() * SO3_R;

//        rotation_matrix = SO3_R.matrix();

        SO3_R = (SO3_R.inverse()) * Sophus::SO3::exp(wv*t);

        SO3_R = SO3_R.inverse();

        rotation_matrix = SO3_R.matrix();

        for (int i(0); i < 3; ++i) {
            for (int j(0); j < 3; ++j) {
                tb2w(i, j) = rotation_matrix(i, j);
            }
        }


        for (int i(0); i < 3; ++i) {
            state_(i) += state_(i + 3) * t + 0.5 * aworld(i) * t * t;
            state_(i + 3) += aworld(i) * t;
        }

        for (int i(0); i < 6; ++i) {
            out(i) = state_(i);
        }


        return out;

    }

protected:

    Eigen::VectorXd state_; // x y z vx vy vz
    std::vector<Eigen::VectorXd> vec_state_;


    Eigen::Affine3d tb2w;

    double time_step_;
};

#endif //QUICKFUSING_IMUINTEGRATE_H
