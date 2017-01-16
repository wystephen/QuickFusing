#pragma once
//
// Created by steve on 17-1-14.
//

#ifndef QUICKFUSING_IMUINTEGRATE_H
#define QUICKFUSING_IMUINTEGRATE_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <thread>
#include <MyError.h>
#include <time_stamp.h>

class ImuIntegrate {
public:
    ImuIntegrate(double error) {
        state_.resize(9);
        state_.setZero();
        time_step_ = 1e-10;
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

        return out;

    }

protected:

    Eigen::VectorXd state_;
    std::vector<Eigen::VectorXd> vec_state_;

    double time_step_;
};

#endif //QUICKFUSING_IMUINTEGRATE_H
