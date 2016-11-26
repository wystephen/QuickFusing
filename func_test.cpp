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
//{
//    Eigen::Matrix4d A, B;
//
//    A.setOnes();
//    std::cout << A << std::endl;
//
//    std::cout << A.array() * 20.0 << std::endl;
//
//    B.setIdentity();
//
//    std::cout << (A) * B << std::endl;
//
//
////    B = Eigen::MatrixXd.diagonal(Eigen::Vector4d(1.0,2.0,3.0,4.0));
//}
//{
//    Eigen::Vector3d q(10,32,20);
//
//    std::cout << q << std::endl;
//
//    q = q.eval() * q.transpose().eval();
//    std::cout << q.transpose() << std::endl;
//
//}
//{
//    Eigen::MatrixXd A,B,C;
//    A.resize(5,5);
//    B.resize(5,5);
//    C.resize(5,5);
//    A.setOnes();
//    std::cout << A << std::endl;
//    B.setIdentity();
//        B(0,0) = 0;
//    B(0,1) = 1;
//    B(1,0) = 1;
//    B(1,1) = 0;
//    A = A * B;
//    std::cout << A << std::endl;
//    A(2,1) = 10;
//
//
//    A =  A.transpose().eval() + 0.5 *  A.eval();
//    std::cout << A << std::endl;
//    A = A * B;
//    std::cout << A<< std::endl;
//}
//{
//    Eigen::VectorXd rnd_vec;
//    rnd_vec.resize(6);
//
//    for(int i(0);i<100;++i)
//    {
//        rnd_vec.setRandom();
//        std::cout << rnd_vec.transpose() << std::endl;
//    }
//}

{
    Eigen::Matrix4d A;
    Eigen::MatrixXd B;

    A.setIdentity();
    B = A;

    std::cout << B << std::endl;
    A.setRandom();
    std::cout << B << std::endl;
    
    
}