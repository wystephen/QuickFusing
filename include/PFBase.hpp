#pragma once
//Create by steve in 16-12-4 at 上午10:23
//
// Created by steve on 16-12-4.
//

#ifndef QUICKFUSING_PFBASE_HPP
#define QUICKFUSING_PFBASE_HPP

#include <random>

#include "MyError.h"
#include "time_stamp.h"

#include <Eigen/Dense>

#include <omp.h>

template<typename T, int state_num, int observe_num>
class PFBase {
public:
    PFBase(int Partical_num) {
        particle_num_ = Partical_num;


        e_.seed(TimeStamp::now());

    }

//    virtual Eigen::VectorXd GetResult();


    /*
     *
     */
//    virtual bool StateTransmition(Eigen::VectorXd input, int MethodType= 0);
//
//    virtual bool Evaluation(Eigen::VectorXd measurement, int MethodType = 0);
//
//    virtual bool Resample(int resample_numint, int MethodType = 0);

    //Auxiliary Tools

    /*
     * Single value Normal probability distribution function.
     */
    double ScalarNormalPdf(double x, double miu, double sigma) {
        try {
            double para1 = (x - miu) * (x - miu) / 2 / sigma / sigma;
            double para2 = (1 / std::sqrt(2 * M_PI) / sigma);

            return para2 * std::exp(-para1);
        } catch (...) {
            //Some error when compute the Single value .

            return 0.000000001;
        }

    }

    double VectorNormalPdf(Eigen::VectorXd x, Eigen::VectorXd miu,
                           Eigen::MatrixXd sigma) {
        try {
            if (x.rows() != miu.rows()) {
                throw ("x.rows is:"
                       + std::to_string(x.rows())
                       + "miu.rows() is"
                       + std::to_string(miu.rows()));
            }

            //ToDo:VectorNormalPdf
            for (int i(0); i < x.rows(); ++i) {

            }

        } catch (std::string &e) {
            //Return zero when size of x is not same to miu.
            std::cout << e << std::endl;
            x.setZero();
            return x;
        } catch (...) {
            MYERROR("Unexpect and unknow error.");
            x.setZero();
            return x;
        }
    }

    /**
    * Resample
    *
    * MethodType:
    * 0: Typical resample method.resample_num is not used in this method.
    * 1: Layer-based resample method.
     * @param resample_num the number of particles generate after resample step.
     * @param MethodType
     * @return
     */
    virtual bool Resample(int resample_num, int MethodType = 0) {
        if (MethodType == 0) {

//            std::vector<Eigen::VectorXd> tmp_vec;
//            std::vector<double> tmp_score;
            Eigen::MatrixXd tmp_vec;
            Eigen::VectorXd tmp_score;
            tmp_vec.resizeLike(p_state_);
            tmp_score.resizeLike(probability_);

            probability_ = probability_ / probability_.sum();


            std::uniform_real_distribution<double> real_distribution(0, 0.9999999);
#pragma omp parallel for
            for (int index = (0); index < p_state_.rows(); ++index) {
                double score = real_distribution(this->e_);
                double tmp_s(score);

                // TOD: Problem is here, but why....?  I know the reason now.
                int i(-1);//TOD: Test it.

                while (score > 0) {
                    i++;
                    score -= probability_(i);
                }
                if (i >= p_state_.rows()) {
                    i = p_state_.rows() - 1;
                    std::cout << probability_.sum() << " is the sum of probability_.";
                    std::cout << tmp_s << "is score" << std::endl;
                }
//                std::cout << p_state_.block(i,0,1,p_state_.cols());
//                tmp_vec.push_back(p_state_.block(i, 0, 1, p_state_.cols()).transpose());
//                tmp_score.push_back(probability_(i));
                for (int k(0); k < tmp_vec.cols(); ++k) {
                    tmp_vec(index, k) = p_state_(i, k);
                }
                tmp_score(index) = probability_(i);

            }

//            for (int index(0); index < probability_.rows(); ++index) {
//                probability_(index) = tmp_score[index];
//                p_state_.block(index, 0, 1, p_state_.cols()) =tmp_vec.block();
//            }
            probability_ = tmp_score;
            p_state_ = tmp_vec;
            if (std::isnan(probability_.sum())) {
                probability_.setOnes();
            }
//            probability_.setOnes();
            probability_ = probability_ / probability_.sum();
        }
    }

protected:
    double particle_num_ = 1000; //

    Eigen::MatrixXd p_state_;//particle filter

    Eigen::VectorXd probability_;//accumulate probability of each particles.


    std::default_random_engine e_;//Global random engine.

//    Eigen::Matrix<double,



};


//PFBase<double,2,4>;

#endif //QUICKFUSING_PFBASE_HPP
