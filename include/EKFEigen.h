//
// Created by steve on 17-6-6.
//

#ifndef INTEGRATINGFUSING_EKFEIGEN_H
#define INTEGRATINGFUSING_EKFEIGEN_H


#include "sophus/so3.h"
#include "sophus/se3.h"

#include "SettingPara.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>


class EKFEigen {
public:
    EKFEigen(SettingPara para) {
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
     * Initial navigation equation parameters.
     * @param u
     * @return
     */
    bool InitNavEq(Eigen::MatrixXd u) {

        long double f_u(0.0), f_v(0.0), f_w(0.0);

        f_u = u.col(0).mean();
        f_v = u.col(1).mean();
        f_w = u.col(2).mean();

        double t_norm = 0.0;
        for (int i(0); i < u.rows(); ++i) {
            t_norm += u.block(i, 0, 1, 3).norm();
        }

        t_norm /= u.rows();

        double roll(std::atan2(-f_v, -f_w));
        double pitch(std::atan2(f_u, std::sqrt(f_v * f_v + f_w * f_w)));

        SO3_rotation_ = Sophus::SO3(Ang2RotMatrix(Eigen::Vector3d(roll, pitch, para_.init_heading1_)));

        Eigen::Vector3d tmp_acc(f_u, f_v, f_w);

//        /// LOOP TO GET RIGHT ANGLE
//        for (int it(0); it < 200; ++it) {
//            double r(atan(tmp_acc(1) / tmp_acc(2))),
//                    p(atan(tmp_acc(0) / std::sqrt(tmp_acc(1) * tmp_acc(1) + tmp_acc(2) * tmp_acc(2))));
//            if (fabs(r) < 0.02 && fabs(p) < 0.02) {
//                break;
//            } else {
//                Sophus::SO3 tmp_so3 = Sophus::SO3(r, p, 0.0);
//                SO3_rotation_ = tmp_so3 * SO3_rotation_;
//                tmp_acc = SO3_rotation_.matrix() * tmp_acc;
//            }
//
//        }
//

        Eigen::Vector3d attitude(SO3_rotation_.log()(0),
                                 SO3_rotation_.log()(1),
                                 SO3_rotation_.log()(2));

        //// JUST FOR DEBUDE (SHOULD BE DELETE AFTER USE)

//        std::cout << "attitude :" << attitude.transpose() << std::endl;
//        std::cout << "SO3 " << SO3_rotation_ << std::endl;
//        tmp_acc = Eigen::Vector3d(f_u, f_v, f_w);
//        Eigen::Matrix3d tm = SO3_rotation_.matrix();
//        tm = tm.transpose().eval();
//        std::cout << "source acc:" << tmp_acc.transpose() << std::endl;
//        std::cout << roll << " --- " << pitch << std::endl;
//        std::cout << tm << std::endl;
//        std::cout << "norm before:" << tmp_acc.norm() << std::endl;
//        tmp_acc = tm * tmp_acc;
//        std::cout << __FILE__ << __LINE__ << "acc after:" << std::endl <<
//                  tmp_acc.transpose() << std::endl;
//        std::cout << "norm after :" << tmp_acc.norm() << std::endl;

        /// DELETE TO HERE

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

    /**
     * Navigation Equation , rotation-first~
     * Use rotation represented by lie group
     * @param x_h (x,y,z,vx,vy,vz)
     * @param u : input of imu (ax,ay,az,wx,wy,wz)
     * @param dt: time interval
     * @return new state of IMu(x,y,z,vx,vy,vz)[rotation matrix is a global value]
     */
    Eigen::VectorXd NavigationEquation(const Eigen::VectorXd &x_h,
                                       const Eigen::VectorXd &u,
                                       double dt) {


        Eigen::VectorXd y;
        y.resize(9);
        y.setZero();
        y = x_h;

        Eigen::Vector3d w_tb(u(3), u(4), u(5));
        w_tb *= dt; // \delta \omega

        if (w_tb.norm() > 1e-18) {

            SO3_rotation_ = SO3_rotation_ * Sophus::SO3::exp(w_tb);
        }

        Eigen::Vector3d g_t(0.0, 0.0, para_.gravity_);//.81);

        Eigen::Matrix3d Rb2t = SO3_rotation_.matrix();
        Eigen::MatrixXd f_t(Rb2t * (u.block(0, 0, 3, 1)));

        Eigen::Vector3d acc_t(f_t + g_t);
//        std::cout << "acc t :" << acc_t.transpose() << std::endl;

        y.block(3, 0, 3, 1) += acc_t * dt;
        y.block(0, 0, 3, 1) += y.block(3, 0, 3, 1) * dt;// + 0.5 * acc_t * dt * dt;


        x_h_ = y;
        return y;
    }

    /**
    *
    * @param q
    * @param u
    * @param dt
    * @return
    */
    bool StateMatrix(const Eigen::VectorXd &u, double dt) {

//        MYCHECK(1);

        Eigen::Matrix3d Rb2t = SO3_rotation_.matrix();


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
     * Correct system state according to dx
     * @param x_in
     * @param dx
     * @return
     */
    Eigen::VectorXd ComputeInternalState(const Eigen::VectorXd &x_in,
                                         Eigen::VectorXd &dx) {


        Eigen::VectorXd x_out = x_in + dx;

        Eigen::Vector3d epsilon(dx.block(6, 0, 3, 1));


        SO3_rotation_ = Sophus::SO3::exp(epsilon) * SO3_rotation_;

//        x_out(6) = SO3_rotation_.log()(0);
//        x_out(7) = SO3_rotation_.log()(1);
//        x_out(8) = SO3_rotation_.log()(2);


        return x_out;

    }


    /**
     * interface for use,
     * @param u
     * @param zupt1
     * @return x_h_ (x,y,z,vx,vy,vz,\theta x,\theta y,\theta z)
     */
    Eigen::VectorXd GetPosition(const Eigen::VectorXd &u,
                                const double &zupt1) {


        last_P_ = P_;
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

            Eigen::MatrixXd Id;
            Id.resize(9, 9);
            Id.setIdentity();

            P_ = (Id - K * H_) * P_;

            x_h_ = ComputeInternalState(x_h_, dx);
        }


        P_ = (P_ * 0.5 + P_.transpose().eval() * 0.5);

        if (std::isnan(P_(0, 0))) {

            std::cout << "Warning: P is Nan" << std::endl;
            P_ = last_P_;
        }


        return x_h_;
    }

    /**
     * Euler angle to rotation matrix.
     * @param ang
     * @return
     */
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

//    double own_g_ = 9.81;


    //P for single foot
    Eigen::Matrix<double, 9, 9> P_;

    Eigen::Matrix<double, 9, 9> last_P_ = Eigen::Matrix<double, 9, 9>::Identity();// C++11 is needed...

    Eigen::Matrix<double, 6, 6> Q_;

    Eigen::Matrix3d R_;

    Eigen::Matrix<double, 3, 9> H_;

    Eigen::Matrix<double, 9, 1> x_h_;

    Eigen::MatrixXd F_;
    Eigen::MatrixXd G_;

    Eigen::MatrixXd K_;


    Sophus::SO3 SO3_rotation_; // rotation matrix of current state of imu.

    Eigen::Matrix<double, 9, 1> dx_; // save the difference between observation value(zero-velocity) and system state(velocity)


    /**
     * for debug.
     */
    std::deque<Eigen::Vector2d> heading_vec_deque_;
    std::deque<double> velocity_deque_;
    int count_move_times_ = 0;

    Eigen::VectorXd last_chage_state_;
    bool last_zupt_ = true;

};

#endif //INTEGRATINGFUSING_EKFEIGEN_H
