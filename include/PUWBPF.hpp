#pragma once
//Create by steve in 16-12-4 at 下午6:12
//
// Created by steve on 16-12-4.
//

#include "PFBase.hpp"

#ifndef QUICKFUSING_PUWBPF_HPP
#define QUICKFUSING_PUWBPF_HPP

template<int uwb_number>
class PUWBPF : public PFBase<double, 2, uwb_number> {
public:
    PUWBPF(int particle_num) {
        PFBase<double, 2, uwb_number>::PFBase(particle_num);
        p_state_.setZero();

        input_noise_sigma_.resize(p_state_.cols());
    }

    bool SetMeasurementSigma(double sigma,int num)
    {
        measurement_sigma_.resize(num);
        for(int i(0);i<measurement_sigma_.rows();++i)
        {
            measurement_sigma_(i) = sigma;
        }
        return true;
    }


    bool SetMeasurementSigma(Eigen::VectorXd sigma_vector)
    {
        measurement_sigma_.resize(sigma_vector.rows());
        measurement_sigma_ = sigma_vector;
        return true;
    }

    bool SetInputNoiseSigma(double sigma)
    {
        Eigen::VectorXd sigma_vector;
        sigma_vector.resize(input_noise_sigma_.rows());

        for(int i(0);i<sigma_vector.rows();++i)
        {
            sigma_vector(i) = sigma;
        }
        SetInputNoiseSigma(sigma_vector);
    }

    bool SetInputNoiseSigma(Eigen::VectorXd sigma_vector) {
        try {
            input_noise_sigma_ = sigma_vector;
        } catch (const std::runtime_error &e) {
            std::cout << "RUNTIME ERROR:" << e << std::endl;
            input_noise_sigma_.setOnes();
            return false;
        } catch (...) {
            if (input_noise_sigma_.size() != sigma_vector.size()) {
                MYERROR("Error in the code that sigma_vector and input_noise_sigma have different size.")
            }
            input_noise_sigma_.setOnes();
            return false;
        }
        return true;
    }

    bool SetBeaconSet(Eigen::MatrixXd beaconset)
    {
        beacon_set_.resizeLike(beaconset);
        beacon_set_ = beacon_set_;
        return true;
    }

    /*
     * State transmission equation.
     */
    bool StateTransmition(Eigen::VectorXd input, int method = 0) {
        if (method == 0)//Method 0:Random move follow the Gaussian distribution(Same sigma).
        {
            double sigma = input_noise_sigma_.mean();
            std::default_random_engine ee_;
            std::normal_distribution<double> normal_distribution(0, sigma);

            for (int i(0); i < p_state_.rows(); ++i) {
                for (int j(0); j < p_state_.cols(); ++j) {
                    p_state_(i,j) += normal_distribution(ee_);
                }
            }

            return true;
        }
    }

    double Evaluation(Eigen::VectorXd state,
                    Eigen::VectorXd measurement)
    {
        double score(0.0);
        try{
            for(int i(0);i<beacon_set_.rows();++i)
            {
                double dis(0.0);
                for(int j(0);j<beacon_set_.cols();++j)
                {
                    dis += std::pow(state(j) - beacon_set_(i,j),2.0);
                }
                dis = std::sqrt(dis);
                score += ScalarNormalPdf(dis,measurement(k),measurement_sigma_(k));

            }
        }catch(...)
        {
            return  0.0;

        }

        return score;
    }


private:
    Eigen::MatrixXd p_state_;//particle filter

    Eigen::VectorXd probability_;//accumulate probability of each particles.


    //Method parameters.

    Eigen::VectorXd input_noise_sigma_;

    Eigen::MatrixXd beacon_set_;

    Eigen::VectorXd measurement_sigma_;


};

#endif //QUICKFUSING_PUWBPF_HPP
