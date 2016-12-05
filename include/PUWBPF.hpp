#pragma once
//Create by steve in 16-12-4 at 下午6:12
//
// Created by steve on 16-12-4.
//

#include "PFBase.hpp"

#ifndef QUICKFUSING_PUWBPF_HPP
#define QUICKFUSING_PUWBPF_HPP

template<int uwb_number>
class PUWBPF:public PFBase<double,2,uwb_number>{
public:
    PUWBPF(int particle_num)
    {
        PFBase<double,2,uwb_number>::PFBase(particle_num);
        p_state_.setZero();

        input_noise_sigma_.resize(p_state_.cols());
    }

    bool StateTransmition(Eigen::VectorXd input,int method = 0)
    {
        if(method == 0)//Method 0:Random move follow the Gaussian distribution.
        {
            std::vector<std::normal_distribution<>> noise_engine_vector_;
            for(int k(0);k<p_state_.cols();++k)
            {
//                noise_engine_vector_.push_back(std::normal_distribution<>(e_));
            }
            for(int i(0);i<p_state_.rows();++i)
            {
                for(int j(0);j<p_state_.cols();++j)
                {

                }
            }

        }
    }


private:
    Eigen::MatrixXd p_state_;//particle filter

    Eigen::VectorXd probability_;//accumulate probability of each particles.


    //Method parameters.

    Eigen::VectorXd input_noise_sigma_;





};

#endif //QUICKFUSING_PUWBPF_HPP
