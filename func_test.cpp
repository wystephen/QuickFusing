//Create by steve in 16-11-23 at 下午10:18
//
// Created by steve on 16-11-23.
//

#include <iostream>

#include <Eigen/Dense>

int main()
//{
//    Eigen::Vector4d q(1.0,2.0,3.0,5.0);
//
//    std::cout << q << std::endl;
//
//    std::cout << q.array().pow(2.0) <<std::endl;
{
    Eigen::Matrix4d A, B;

    A.setOnes();
    std::cout << A << std::endl;

    std::cout << A.array() * 20.0 << std::endl;

    B.setIdentity();

    std::cout << (A) * B << std::endl;
}