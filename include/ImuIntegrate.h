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


//#include "sophus/so3.h"
//#include "sophus/se3.h"

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


//        std::cout << "a is: " << a.transpose() << std::endl;
//        std::cout << "w v is : " << wv.transpose() << std::endl;


        Eigen::Matrix3d rotation_matrix;
        for (int i(0); i < 3; ++i) {
            for (int j(0); j < 3; ++j) {
                rotation_matrix(i, j) = tb2w(i, j);
            }
        }


        Eigen::Vector4d q = dcm2q(rotation_matrix);

        Eigen::Vector3d w_tb(wv);
        double dt = t;
        double v(w_tb.norm() * dt);
        if (std::fabs(v) > 1e-8) {
            double P(w_tb(0) * dt * 0.5);
            double Q(w_tb(1) * dt * 0.5);
            double R(w_tb(2) * dt * 0.5);

            Eigen::Matrix4d OMEGA;

            OMEGA.setZero();

            OMEGA(0, 1) = R;
            OMEGA(0, 2) = -Q;
            OMEGA(0, 3) = P;

            OMEGA(1, 0) = -R;
            OMEGA(1, 2) = P;
            OMEGA(1, 3) = Q;

            OMEGA(2, 0) = Q;
            OMEGA(2, 1) = -P;
            OMEGA(2, 3) = R;

            OMEGA(3, 0) = -P;
            OMEGA(3, 1) = -Q;
            OMEGA(3, 2) = -R;

            q = (std::cos(v / 2.0) * Eigen::Matrix4d::Identity() +
                 2.0 / v * sin(v / 2.0) * OMEGA) * q;

            q /= q.norm();
        }

        rotation_matrix = q2dcm(q);


        for (int i(0); i < 3; ++i) {
            for (int j(0); j < 3; ++j) {
                tb2w(i, j) = rotation_matrix(i, j);
            }
        }
        Eigen::VectorXd aworld = rotation_matrix * a;
        aworld(2) += 9.8173;

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
    /*
        * Rotation matrix to quanternions.
        */
    Eigen::Vector4d dcm2q(Eigen::Matrix3d R) {
//        MYCHECK(1);
        double T(1.0 + R(0, 0) + R(1, 1) + R(2, 2));

        double qw(0.0), qx(0.0), qy(0.0), qz(0.0);
        double S(0.0);


        try {
            // 1e-3  ==>>>  fabs(T) != 0
            if (fabs(T) > 1e-3) {
                S = 0.5 / sqrt(fabs(T));

                qw = 0.25 / S;
                qx = (R(2, 1) - R(1, 2)) * S;
                qy = (R(0, 2) - R(2, 0)) * S;
                qz = (R(1, 0) - R(0, 1)) * S;

            } else {
                if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
                    S = sqrt(1 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;

                    qw = (R(2, 1) - R(1, 2)) / S;
                    qx = 0.25 * S;
                    qy = (R(0, 1) + R(1, 0)) / S;
                    qz = (R(0, 2) + R(2, 0)) / S;
                } else if (R(1, 1) > R(2, 2)) {
                    S = sqrt(1 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;

                    qw = (R(0, 2) - R(2, 0)) / S;
                    qx = (R(0, 1) + R(1, 0)) / S;
                    qy = 0.25 * S;
                    qz = (R(1, 2) + R(2, 1)) / S;
                } else {

                    S = sqrt(1 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;

                    qw = (R(1, 0) - R(0, 1)) / S;
                    qx = (R(0, 2) + R(2, 0)) / S;
                    qy = (R(1, 2) + R(2, 1)) / S;
                    qz = 0.25 * S;

                }

            }

            Eigen::Vector4d quart(qx, qy, qz, qw);
//            std::cout << "quart:" << quart << "norm:" << quart.norm() << std::endl;
            quart /= quart.norm();

            return quart;

        } catch (...) {
            std::cout << "THERE ARE SOME ERROR!" << std::endl;
            return Eigen::Vector4d(0, 0, 0, 1.0);
        }
    }


    /*
     *Quanternions to rotation matrix.
     */
    Eigen::Matrix3d q2dcm(Eigen::Vector4d q) {
//        MYCHECK(1);

        Eigen::VectorXd p;
        p.resize(6);
        p.setZero();

        p.block(0, 0, 4, 1) = q.array().pow(2.0);

        p(4) = p(1) + p(2);

        if (fabs(p(0) + p(3) + p(4)) > 1e-18) {
            p(5) = 2.0 / (p(0) + p(3) + p(4));

        } else {
            p(5) = 0.0;
        }

        Eigen::Matrix3d R;
        R.setZero();

        R(0, 0) = 1 - p(5) * p(4);
        R(1, 1) = 1 - p(5) * (p(0) + p(2));
        R(2, 2) = 1 - p(5) * (p(0) + p(1));

        p(0) = p(5) * q(0);
        p(1) = p(5) * q(1);
        p(4) = p(5) * q(2) * q(3);
        p(5) = p(0) * q(1);

        R(0, 1) = p(5) - p(4);
        R(1, 0) = p(5) + p(4);

        p(4) = p(1) * q(3);
        p(5) = p(0) * q(2);

        R(0, 2) = p(5) + p(4);
        R(2, 0) = p(5) - p(4);

        p(4) = p(0) * q(3);
        p(5) = p(1) * q(2);

        R(1, 2) = p(5) - p(4);
        R(2, 1) = p(5) + p(4);

        return R;
    }


    Eigen::VectorXd state_; // x y z vx vy vz
    std::vector<Eigen::VectorXd> vec_state_;


    Eigen::Affine3d tb2w;

    double time_step_;
};

#endif //QUICKFUSING_IMUINTEGRATE_H
