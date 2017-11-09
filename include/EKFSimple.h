//
// Created by steve on 17-6-11.
//

#ifndef INTEGRATINGFUSING_EKFSIMPLE_H
#define INTEGRATINGFUSING_EKFSIMPLE_H

#include "sophus/so3.h"
#include "sophus/se3.h"

#include "SettingPara.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>


class EKFSimple {
public:
    EKFSimple(SettingPara para) {
        para_ = para;

        R_ = Eigen::Matrix3d::Zero();

        P_.resize(9, 9);
        P_.setZero();

        Q_.resize(6, 6);
        Q_.setZero();

        H_.resize(3, 9);
        H_.setZero();


        InitialFilter();

        x_h_.resize(9, 1);
        x_h_.setZero();

        K_.resize(9, 3);
        K_.setIdentity();
    }


    /**
  * Initial parameters in navigation equation.
  * @param u
  * @return
  */
    bool InitNavEq(Eigen::MatrixXd u) {

        double f_u(0.0), f_v(0.0), f_w(0.0);

        f_u = u.col(0).mean();
        f_v = u.col(1).mean();
        f_w = u.col(2).mean();

        double t_norm = 0.0;
        for (int i(0); i < u.rows(); ++i) {
            t_norm += u.block(i, 0, 1, 3).norm();
        }

        t_norm /= u.rows();
//        double roll(std::atan2(-f_v, -std::sqrt(f_w * f_w + f_u * f_u)));
        double roll(std::atan2(-f_v, -f_w));
        double pitch(std::atan2(f_u, std::sqrt(f_v * f_v + f_w * f_w)));


        Eigen::Vector3d attitude(roll, pitch, para_.init_heading1_);
//

        std::cout << "roll : " << roll << "pitch: " << pitch << std::endl;
        rotation_matrix_ = Ang2RotMatrix(attitude);
//        rotation_matrix_ = rotation_matrix_.transpose();
        rotation_matrix_.transposeInPlace();


        Eigen::Vector3d source_acc(f_u, f_v, f_w);

        std::cout << "source :" << source_acc.transpose() << std::endl;

        source_acc = rotation_matrix_ * source_acc;

        std::cout << " after : " << source_acc.transpose() << std::endl;


        x_h_.block(0, 0, 3, 1) = para_.init_pos1_;
        x_h_.block(6, 0, 3, 1) = attitude;


        return true;
    }

    /**
 * Initial Filter .(only run in construct function.)
 * @return
 */
    bool InitialFilter() {

        for (int i(0); i < 3; ++i) {
            P_(i, i) = para_.sigma_initial_pos1_(i) * para_.sigma_initial_pos1_(i);
            P_(i + 3, i + 3) = para_.sigma_initial_vel1_(i) * para_.sigma_initial_vel1_(i);
            P_(i + 6, i + 6) = para_.sigma_initial_att1_(i) * para_.sigma_initial_att1_(i);

            R_(i, i) = para_.sigma_vel_(i) * para_.sigma_vel_(i);

            Q_(i, i) = para_.sigma_acc_(i) * para_.sigma_acc_(i);
            Q_(i + 3, i + 3) = para_.sigma_gyro_(i) * para_.sigma_gyro_(i);

            H_(i, i + 3) = 1.0;
        }


        return true;
    }

    Eigen::VectorXd NavigationEquation(Eigen::VectorXd x_h,
                                       Eigen::VectorXd u,
                                       double dt) {

//        MYCHECK(1);


        Eigen::VectorXd y;
        y.resize(9);
        y.setZero();

        Eigen::Vector3d w_tb(u(3), u(4), u(5));

        if (fabs(w_tb.norm()) > 1e-18) {


            Eigen::Matrix3d ang_rate_matrix;
            ang_rate_matrix.setZero();

            w_tb *= dt;



//            ang_rate_matrix << 0.0, -w_tb(2), w_tb(1),
//                    w_tb(2), 0.0, w_tb(0),
//                    -w_tb(1), w_tb(0), 0.0;
            Eigen::Vector3d alpha(0,0,0);
            alpha = w_tb/w_tb.norm();
            double phi(w_tb.norm());
            Eigen::Matrix3d alpha_hat;
            alpha_hat<< 0.0,-alpha(2),alpha(1),
                    alpha(2),0.0,-alpha(0),
                    -alpha(1),alpha(0),0.0;




            rotation_matrix_  = rotation_matrix_ * (cos(phi)*Eigen::Matrix3d::Identity()+
                    (1-cos(phi))*alpha*alpha.transpose()+sin(phi)*alpha_hat);


        } else {
            /*
             * Need not do any thing.
             */
//            quaterniond_ = q;
        }

        //---------------
        Eigen::Vector3d g_t(0, 0, 9.81);
//        g_t = g_t.transpose();

        Eigen::Matrix3d Rb2t = rotation_matrix_;
        Eigen::MatrixXd f_t(Rb2t * (u.block(0, 0, 3, 1)));

        Eigen::Vector3d acc_t(f_t + g_t);

        Eigen::MatrixXd A, B;

        A.resize(6, 6);
        A.setIdentity();

        A(0, 3) = dt;
        A(1, 4) = dt;
        A(2, 5) = dt;

        B.resize(6, 3);
        B.setZero();
        Eigen::Matrix3d tmp;

//        std::cout << B.rows() << " x " << B.cols() << std::endl;
//        tmp.setZero();
//        B.block(0, 0, 3, 3) = tmp;
        B.block(3, 0, 3, 3) = Eigen::Matrix3d::Identity() * dt;

//        MYCHECK(1);
        y.block(0, 0, 6, 1) = A * (x_h.block(0, 0, 6, 1)) +
                              B * acc_t;

        x_h_ = y;
//        MYCHECK(1);
        return y;
    }

    /**
    *
    * @param q
    * @param u
    * @param dt
    * @return
    */
    bool StateMatrix(Eigen::VectorXd u, double dt) {

//        MYCHECK(1);

        Eigen::Matrix3d Rb2t = rotation_matrix_;//.matrix();


        Eigen::Vector3d f_t(Rb2t * (u.block(0, 0, 3, 1)));

        Eigen::Matrix3d St;
        St.setZero();

        St(0, 1) = -f_t(2);
        St(0, 2) = f_t(1);

        St(1, 0) = f_t(2);
        St(1, 2) = f_t(0);

        St(2, 0) = -f_t(1);
        St(2, 1) = f_t(0);

        Eigen::MatrixXd Fc;
        Fc.resize(9, 9);
        Fc.setZero();

        Fc.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
        Fc.block(3, 6, 3, 3) = St;

        Eigen::MatrixXd Gc;
        Gc.resize(9, 6);
        Gc.setZero();

        Gc.block(3, 0, 3, 3) = Rb2t;
        Gc.block(6, 3, 3, 3) = -Rb2t;


        Eigen::MatrixXd Id;
        Id.resize(9, 9);
        Id.setIdentity();

        F_ = Id + (Fc * dt);
        G_ = Gc * dt;

        return true;

    }


    /**
     * Plus the error of obvious into the state.
     * @param x_in current state in prior
     * @param dx
     * @param q_in current quantanien
     * @return
     */
    Eigen::VectorXd ComputeInternalState(Eigen::VectorXd x_in,
                                         Eigen::VectorXd dx) {


        Eigen::VectorXd x_out = x_in + dx;

        Eigen::Vector3d w_tb(dx.block(6, 0, 3, 1));


        Eigen::Matrix3d ang_rate_matrix;
        ang_rate_matrix.setZero();

        ang_rate_matrix <<
                        0.0, -w_tb(2), w_tb(1),
                        w_tb(2), 0.0, -w_tb(0),
                        -w_tb(1), w_tb(0), 0.0;

//        rotation_matrix_ = ((2 * Eigen::Matrix3d::Identity() + ang_rate_matrix)*
//                            (2 * Eigen::Matrix3d::Identity() - ang_rate_matrix).inverse) * rotation_matrix_;
        rotation_matrix_ = (Eigen::Matrix3d::Identity()-ang_rate_matrix)*rotation_matrix_;

//        rotation_matrix_ = Sophus::SO3::exp(w_tb).matrix() * rotation_matrix_;
//
//        x_out(6) = SO3_rotation_.log()(0);
//        x_out(7) = SO3_rotation_.log()(1);
//        x_out(8) = SO3_rotation_.log()(2);


        return x_out;

    }

    Eigen::VectorXd GetPosition(Eigen::VectorXd u, double zupt1) {

        x_h_ = NavigationEquation(x_h_, u, para_.Ts_);

        StateMatrix(u, para_.Ts_);

        P_ = (F_ * (P_)) * (F_.transpose().eval()) +
             (G_ * Q_ * G_.transpose().eval());
        if (zupt1 > 0.5) {
            Eigen::Vector3d z(-x_h_.block(3, 0, 3, 1));

            Eigen::MatrixXd K;
            K = P_ * H_.transpose().eval() * (H_ * P_ * H_.transpose().eval() + R_).inverse();

            Eigen::VectorXd dx = K * z;
            dx_ = dx;

            Eigen::Matrix<double,9,9> Id;
            Id.setIdentity();

            P_ = (Id - K * H_) * P_;

            x_h_ = ComputeInternalState(x_h_, dx);
        }


        P_ = (P_ * 0.5 + P_.transpose().eval() * 0.5);


        return x_h_;
    }

    Eigen::VectorXd OutputAxis() {

        Eigen::VectorXd axis_vec(12);
        // center point
        for (int i(0); i < 3; ++i) {
            axis_vec(i) = x_h_(i);
        }

        Eigen::Matrix3d rt = rotation_matrix_;
        // offset of each axis
        for (int i(0); i < 3; ++i) {
            Eigen::Vector3d at;
            at.setZero();
            at(i) = 1.0;

            at = rotation_matrix_ * at;
            for (int j(0); j < 3; ++j) {
                axis_vec(2 + i * 3 + j) = at(j);
            }

        }

        return axis_vec;


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


public:
    //Parameters in here.
    SettingPara para_;
private:


    //P for single foot
    Eigen::Matrix<double, 9, 9> P_;

    Eigen::Matrix<double, 6, 6> Q_;

    Eigen::Matrix3d R_;

    Eigen::Matrix<double, 3, 9> H_;

    Eigen::Matrix<double, 9, 1> x_h_;

    Eigen::MatrixXd F_;
    Eigen::MatrixXd G_;

    Eigen::MatrixXd K_;



//    Eigen::Vector4d quat_;

//    Eigen::Quaterniond quaterniond_;

//    Sophus::SO3 SO3_rotation_;
    Eigen::Matrix3d rotation_matrix_;

    Eigen::MatrixXd dx_;


    std::deque<Eigen::Vector2d> heading_vec_deque_;
    std::deque<double> velocity_deque_;
    int count_move_times_ = 0;


    Eigen::VectorXd last_chage_state_;

    bool last_zupt_ = true;
};


#endif //INTEGRATINGFUSING_EKFSIMPLE_H
