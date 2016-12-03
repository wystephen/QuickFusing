#pragma once
//Create by steve in 16-12-3 at 下午1:25
//
// Created by steve on 16-12-3.
//

#ifndef QUICKFUSING_RANGEKF_HPP
#define QUICKFUSING_RANGEKF_HPP

//#include <cmath>


class SingleValueFilter {
public:
    SingleValueFilter(double EstimateCov = 0.5, double MeasureCov = 0.5) {

    }

    /*
     * One step filter,output the value after filter.
     */
    double filter(double val)
    {
        K_ = EstimateCov_ *
                std::sqrt(1/ (EstimateCov_*EstimateCov_+MeasureCov_*MeasureCov_));
        Estimate_ = Estimate_ + K_ * (val-Estimate_);

        EstimateCov_ = std::sqrt(1-K_) * EstimateCov_;
        MeasureCov_ = std::sqrt(1-K_) * MeasureCov_;

        return Estimate_;
    }


private:
    double EstimateCov_ = 0.5;//Covariance for estimate value.

    double MeasureCov_ = 0.5;//Covariance for measurement value.

    double K_;      //Kalman gain

    double Estimate_; //Estimate value.





};



#endif //QUICKFUSING_RANGEKF_HPP
