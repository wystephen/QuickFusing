#pragma once
//Create by steve in 16-12-4 at 上午10:23
//
// Created by steve on 16-12-4.
//

#ifndef QUICKFUSING_PFBASE_HPP
#define QUICKFUSING_PFBASE_HPP

#include "MyError.h"
#include <Eigen/Dense>

template <typename T,int state_num,int observe_num>
class PFBase{
public:
    virtual PFBase(double Partical_num);

    virtual Eigen::VectorXd GetResult();


    virtual bool StateTransmition(Eigen::Vector<double,state_num> input);

    virtual bool Evaluation(Eigen::Vector<double,observe_num> measurement);

    virtual bool Resample(int MethodType,int resample_num);

    //Auxiliary Tools

    /*
     * Single value Normal probability distribution function.
     */
    double ScalarNormalPdf(double x,double miu,double sigma)
    {
        try {
            double para1 = (x - miu) * (x - miu) / 2 / sigma / sigma;
            double para2 = (1 / std::sqrt(2 * M_PI) / sigma);

            return para2 * std::log(-para1);
        }catch (...)
        {
            //Some error when compute the Single value .

            return 0.0;
        }

    }

    double VectorNormalPdf(Eigen::VectorXd x,Eigen::VectorXd miu,
    Eigen::MatrixXd sigma)
    {
        try{
            if(x.rows() != miu.rows())
            {
                throw("x.rows is:"
                + std::to_string(x.rows())
                +"miu.rows() is"
                + std::to_string(miu.rows()));
            }

            //ToDo:VectorNormalPdf

        }catch(std::string &e) {
            //Return zero when size of x is not same to miu.
            std::cout << e << std::endl;
            x.setZero();
            return x;
        }catch (...)
        {
           MYERROR("Unexpect and unknow error.");
            x.setZero();
            return x;
        }
    }


private:
    double particle_num_ = 1000; //

//    Eigen::Matrix<double,



};
#endif //QUICKFUSING_PFBASE_HPP
