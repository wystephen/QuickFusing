//
// Created by steve on 17-6-26.
//

#ifndef INTEGRATINGFUSING_EKFR_H
#define INTEGRATINGFUSING_EKFR_H

#include "SettingPara.h"

class EKFR {
public:
    EKFR(SettingPara initial_para) {
        para_ = initial_para;

        C_ = C_pre_ = Eigen::Matrix3d::Identity();

        P_.resize(9, 9);
        P_.setZero();

        Q_.resize(9, 9);
        Q_.setZero();

        H_.resize(3, 9);
        H_.setZero();
        F_.setZero();
        std::cout << "begin" << std::endl;
        for (int i(0); i < 3; ++i) {
//            P_(i, i) = para_.sigma_initial_pos1_(i) * para_.sigma_initial_pos1_(i);
//            P_(i + 3, i + 3) = para_.sigma_initial_vel1_(i) * para_.sigma_initial_vel1_(i);
//            P_(i + 6, i + 6) = para_.sigma_initial_att1_(i) * para_.sigma_initial_att1_(i);

            R_(i, i) = para_.sigma_vel_(i) * para_.sigma_vel_(i) * para_.Ts_;

            Q_(i+6, i+6) = para_.sigma_acc_(i) * para_.sigma_acc_(i)*para_.Ts_;
            Q_(i , i ) = para_.sigma_gyro_(i) * para_.sigma_gyro_(i)*para_.Ts_;

            H_(i, i + 3) = 1.0;
        }

        x_h_.resize(9, 1);
        x_h_.setZero();

        K_.resize(9, 3);
        K_.setIdentity();
    }

    bool InitNavEq(Eigen::MatrixXd u) {
        long double f_u(0.0), f_v(0.0), f_w(0.0);

        f_u = u.col(0).mean();
        f_v = u.col(1).mean();
        f_w = u.col(2).mean();

//        double roll(std::atan2(-f_v, -f_w));
//        double pitch(std::atan2(f_u, std::sqrt(f_v * f_v + f_w * f_w)));
        double roll = std::atan(f_v/f_w);
        double pitch = -std::asin(f_u/std::sqrt(f_u*f_u+f_v*f_v+f_w*f_w));

//        C_pre_ = C_ = Ang2RotMatrix(Eigen::Vector3d(roll, pitch, para_.init_heading1_));
        C_pre_ = C_ = Ang2RotMatrix(pitch,roll,para_.init_heading1_);
        Eigen::Vector3d acc(f_u,f_v,f_w);

        std::cout << "acc src:" << acc.transpose() << std::endl;
        std::cout << "acc after:" << C_ * acc << std::endl;
        if(std::isnan(C_.sum()))
        {
//            std::cout << "error in initial navigation equation" << std::endl;
        }

    }

    Eigen::VectorXd GetPosition(Eigen::VectorXd u, double zupt_flag, double dt = -10.0) {

        if (dt < 0.0) {
            dt = para_.Ts_;
        }

        if(std::isnan(x_h_.sum()))
        {
//            std::cout << "x_h_ is nan" << std::endl;
        }


        Eigen::Matrix3d ang_rate_matrix = hat(u.block(3, 0, 3, 1));

        C_ = C_pre_ * (2 * Eigen::Matrix3d::Identity() + (ang_rate_matrix * dt)) *
             (2 * Eigen::Matrix3d::Identity() - (ang_rate_matrix * dt)).inverse();

        Eigen::Vector3d acc = 0.5 * (C_ + C_pre_) * u.block(0, 0, 3, 1);

        x_h_.block(3, 0, 3, 1) = x_h_.block(3, 0, 3, 1) + (acc - Eigen::Vector3d(0, 0, 9.81)) * dt;
//        std::cout << "acc: " << acc - Eigen::Vector3d(0.0,0.0,9.81) << std::endl;

        x_h_.block(0, 0, 3, 1) = x_h_.block(0, 0, 3, 1) + x_h_.block(3, 0, 3, 1) * dt;

        Eigen::Matrix3d S = hat(acc);

        F_.setIdentity();
        F_.block(3, 6, 3, 3) = dt * Eigen::Matrix3d::Identity();
        F_.block(6, 0, 3, 3) = -dt * S;

        P_ = F_ * P_ * F_.transpose().eval() + Q_;

        if (zupt_flag > 0.5) {
            K_ = (P_ * H_.transpose().eval()) * (H_ * P_ * H_.transpose().eval() + R_);

            Eigen::Matrix<double, 9, 1> delta_x,t_delta_x;
            t_delta_x = K_ * x_h_.block(3, 0, 3, 1);
            delta_x.block(0,0,3,1) = t_delta_x.block(3,0,3,1);
            delta_x.block(3,0,3,1) = t_delta_x.block(6,0,3,1);
            delta_x.block(6,0,3,1) = t_delta_x.block(0,0,3,1);



            P_ = (Eigen::Matrix<double, 9, 9>::Identity() - K_ * H_) * P_;

            x_h_.block(0, 0, 6, 1) -= delta_x.block(0, 0, 6, 1);

            Eigen::Matrix3d ang_matrix = hat(delta_x.block(6, 0, 3, 1));

            C_ = (2 * Eigen::Matrix3d::Identity() + ang_matrix) *
                 (2 * Eigen::Matrix3d::Identity() - ang_matrix).inverse().eval() * C_;
            if(std::isnan(C_.sum()))
            {
//                std::cout << "error in initial navigation equation" << std::endl;
            }
            C_pre_ = C_;



        }
        P_ = (P_+P_.transpose().eval())*0.5;


        return x_h_;


    }


    Eigen::Matrix3d hat(const Eigen::Vector3d &a) {
        Eigen::Matrix3d A;
        A << 0.0, -a(2), a(1),
                a(2), 0.0, -a(0),
                -a(1), a(0), 0.0;
        return A;
    }

    inline Eigen::Matrix3d Ang2RotMatrix(Eigen::Vector3d ang) {
        double cr(cos(ang(0)));
        double sr(sin(ang(0)));

        double cp(cos(ang(1)));
        double sp(sin(ang(1)));

        double cy(cos(ang(2)));
        double sy(sin(ang(2)));

        Eigen::Matrix3d R3;
        R3 << cy * cp, sy * cp, -sp,
                -sy * cr + cy * sp * sr, cy * cr + sy * sp * sr, cp * sr,
                sy * sr + cy * sp * cr, -cy * sr + sy * sp * cr, cp * cr;

        return R3;

    }

    inline Eigen::Matrix3d Ang2RotMatrix(double pitch,double roll,double yaw)
    {
        double cp = std::cos(pitch);
        double sp = std::sin(pitch);

        double cr = std::cos(roll);
        double sr = std::sin(roll);

        double cy = std::cos(yaw);
        double sy = std::sin(yaw);

        Eigen::Matrix3d C;
        C<< cp*cy,(sr*sp*cy)-(cr*sy),(cr*sp*cy+sr*sy),
        cp*sy,(sr*sp*sy)+(cr*cy),(cr*sp*sy)-(sr*cy),
        -sp,sr*cp,cr*cp;
        return C;
    }

    Eigen::Matrix<double, 9, 9> P_;

    Eigen::Matrix<double, 9, 9> Q_;

    Eigen::Matrix3d C_;
    Eigen::Matrix3d C_pre_;
    Eigen::Matrix3d R_;
    Eigen::Matrix<double, 3, 9> H_;

    Eigen::Matrix<double, 9, 1> x_h_;

    Eigen::Matrix<double, 9, 9> F_;
    Eigen::MatrixXd G_;

    Eigen::MatrixXd K_;

    SettingPara para_;
};

#endif //INTEGRATINGFUSING_EKFR_H
