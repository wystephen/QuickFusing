//
// Created by steve on 17-1-14.
//


#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"

int main()
{

    std::cout.precision(20);
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2.0,Eigen::Vector3d(0,0,1)).toRotationMatrix();

    Sophus::SO3 SO3_R(R);
    Sophus::SO3 SO3_v(0,0,M_PI/2);

    std::cout << "SO3_R:" << SO3_R << std::endl;
    std::cout << "SO3_V:" << SO3_v << std::endl;
}

