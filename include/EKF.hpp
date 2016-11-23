#pragma once
//
// Created by steve on 16-11-23.
//

//Create by steve in 16-11-23 at 下午8:02

#include "SettingPara.h"


#ifndef QUICKFUSING_EKF_HPP
#define QUICKFUSING_EKF_HPP


class EKF {
public:
    EKF(SettingPara para){



    }


protected:


private:
    //Parameters in here.
    SettingPara para_;

    //P for single foot
    Eigen::MatrixXd<9,9> P_;

    Eigen::MatrixXd<6,6> Q_;

    Eigen::MatrixXd<3,9> H_;

    Eigen::VectorXd<9> x_h_;






};


#endif //QUICKFUSING_EKF_HPP
