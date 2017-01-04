#pragma once
//Create by steve in 16-12-4 at 下午6:12
//
// Created by steve on 16-12-4.
//

#include "PFBase.hpp"

#ifndef QUICKFUSING_PUWBPF_HPP
#define QUICKFUSING_PUWBPF_HPP

#define ISDEBUG false

template<int uwb_number>
class PUWBPF : public PFBase<double, 2, uwb_number> {
public:
    PUWBPF(int particle_num) : PFBase<double, 2, uwb_number>(particle_num) {
//        PFBase(particle_num);
        try {
            this->p_state_.resize(particle_num, 2);

            this->p_state_.setZero();
            this->probability_.resize(particle_num);
            this->probability_.setOnes();
            this->probability_ = this->probability_ / this->probability_.sum();
            input_noise_sigma_.resize(this->p_state_.cols());
        } catch (...) {
            MYERROR("PUWBPF initial error.");
        }

    }

    bool SetMeasurementSigma(double sigma, int num = uwb_number) {
        try {
            measurement_sigma_.resize(num);
            for (int i(0); i < measurement_sigma_.rows(); ++i) {
                measurement_sigma_(i) = sigma;
            }
        } catch (...) {
            MYERROR("ERROR")
        }

        return true;
    }


    bool SetMeasurementSigma(Eigen::VectorXd sigma_vector) {
        MYCHECK(ISDEBUG);
        measurement_sigma_.resize(sigma_vector.rows());
        measurement_sigma_ = sigma_vector;
        return true;
    }

    bool SetInputNoiseSigma(double sigma) {
        MYCHECK(ISDEBUG);
        Eigen::VectorXd sigma_vector;
        sigma_vector.resize(input_noise_sigma_.rows());

        for (int i(0); i < sigma_vector.rows(); ++i) {
            sigma_vector(i) = sigma;
        }
        SetInputNoiseSigma(sigma_vector);
        return true;
    }

    /**
     *
     * @param sigma_vector
     * @return
     */
    bool SetInputNoiseSigma(Eigen::VectorXd sigma_vector) {
        MYCHECK(ISDEBUG);
        try {
            input_noise_sigma_ = sigma_vector;
        } catch (const std::runtime_error &e) {
            std::cout << "RUNTIME ERROR:" << e.what() << std::endl;
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

    bool SetBeaconSet(Eigen::MatrixXd beaconset) {
        MYCHECK(ISDEBUG);
        beacon_set_.resizeLike(beaconset);
        beacon_set_ = beaconset;
        std::cout << "beaconset:" << std::endl;
        std::cout << beacon_set_ << std::endl;
        return true;
    }

    /**
     * State transmition function.
     *
     * @param input :
     * @param MethodType
     * @return
     */
    bool StateTransmition(Eigen::VectorXd input, int MethodType = 0) {
        MYCHECK(ISDEBUG);
        if (MethodType == 0)//Method 0:Random move follow the Gaussian distribution(Same sigma).
        {
            double sigma = input_noise_sigma_.mean();

//            try{
//                std::cout << input.size() << std::endl;
//
//            }catch (std::exception &e)
//            {
//                std::cout << e.what() << std::endl;
//            }
            MYCHECK(ISDEBUG);
            std::default_random_engine ee_;
            std::normal_distribution<double> normal_distribution(0, sigma);
            MYCHECK(ISDEBUG);
            for (int i(0); i < this->p_state_.rows(); ++i) {
                for (int j(0); j < this->p_state_.cols(); ++j) {
                    this->p_state_(i, j) += normal_distribution(ee_);
//                    MYCHECK(ISDEBUG);
                }
            }
            MYCHECK(ISDEBUG);
//            std::cout << p_state_<<std::endl;
//            std::cout << "a" << std::endl;
            return true;
        }
    }

      /**
      *
      * Evaluation function.
      * Input state and measurement data,and compute a score.
      *
      * @param measurement
      * @param MethodType
      * @return
      */
    bool Evaluation(Eigen::VectorXd measurement, int MethodType = 0) {
        MYCHECK(ISDEBUG);
        if (MethodType == 0) {
            for (int i(0); i < this->p_state_.rows(); ++i) {
//                std::cout << "endl:" << std::endl;
//                std::cout << p_state_.block(i, 0, 1, p_state_.cols())<< " here   " << std::endl;
//                std::cout << measurement << "here 2 " << std::endl;

                this->probability_(i) *= EvaluationSingle(
                        this->p_state_.block(i, 0, 1, this->p_state_.cols()).transpose(),
                        measurement);
            }
        }

        ///normalize probability.
        this->probability_ /= this->probability_.sum();
        return true;

    }

    /**
     * @brief
     *
     * @param state
     * @param measurement vectorXd,dimension equal to the number of uwb.
     * @return
     */
    double EvaluationSingle(Eigen::VectorXd state,
                            Eigen::VectorXd measurement) {
        MYCHECK(ISDEBUG);
        double score(1.0);
//        std::cout << beacon_set_.rows()<<" : " << beacon_set_.cols() << std::endl;
//        std::cout << state.rows() <<" :     :"<<state.cols() << std::endl;
        try {
            for (int i(0); i < beacon_set_.rows(); ++i) {
                double dis(0.0);
                for (int j(0); j < beacon_set_.cols() - 1; ++j) {
                    dis += std::pow(state(j) - beacon_set_(i, j), 2.0);
                }
                dis += std::pow(2.14 - beacon_set_(i, 2), 2.0);//TODO: Change this->one.
                dis = std::sqrt(dis);
                MYCHECK(ISDEBUG);

                score *= (this->ScalarNormalPdf(dis, measurement(i), measurement_sigma_(i)) + 1e-50);
//                std::cout << score << ";:::" <<
//                          dis << " :"
//                          << measurement(i) << ":" << measurement_sigma_(i) << "dddd" << std::endl;
            }
        } catch (...) {
            return 0.0;
        }


        return score;
    }


    /**
    * Resample
    *
    * MethodType:
    * 0: Typical resample method.resample_num is not used in this->method.
    * 1: Layer-based resample method.
     * @param resample_num the number of particles generate after resample step.
     * @param MethodType
     * @return
     */
//     bool Resample(int resample_num,int MethodType = 0);
    bool Resample(int resample_num, int MethodType = 0) {
        MYCHECK(ISDEBUG);
        if (MethodType == 0) {

            std::vector<Eigen::VectorXd> tmp_vec;
            std::vector<double> tmp_score;

            this->probability_ = this->probability_ / this->probability_.sum();


            std::uniform_real_distribution<double> real_distribution(0, 0.9999999);
            MYCHECK(ISDEBUG);
            for (int index(0); index < this->p_state_.rows(); ++index) {
                double score = real_distribution(this->e_);
                double tmp_s(score);

                // TOD: Problem is here, but why....?  I know the reason now.
                int i(-1);//TOD: Test it.

                MYCHECK(ISDEBUG);
                while (score > 0) {
                    i++;
                    score -= this->probability_(i);
                }
                if (i >= this->p_state_.rows()) {
                    i = this->p_state_.rows() - 1;
                    std::cout << this->probability_.sum() << " is the sum of probability_.";
                    std::cout << tmp_s << "is score" << std::endl;
                }
                MYCHECK(ISDEBUG);
//                std::cout << p_state_.block(i,0,1,p_state_.cols());
                tmp_vec.push_back(this->p_state_.block(i, 0, 1, this->p_state_.cols()).transpose());
                MYCHECK(ISDEBUG);
                tmp_score.push_back(this->probability_(i));

            }
            MYCHECK(ISDEBUG);

            for (int index(0); index < this->probability_.rows(); ++index) {
                this->probability_(index) = tmp_score[index];
                this->p_state_.block(index, 0, 1, this->p_state_.cols()) = tmp_vec[index].transpose();
            }
            if (isnan(this->probability_.sum())) {
                this->probability_.setOnes();
            }
//            probability_.setOnes();
            this->probability_ = this->probability_ / this->probability_.sum();
        }
    }

    /**
     * Average with weight.
     * @param MethodType
     * @return
     */
    Eigen::VectorXd GetResult(int MethodType = 0) {
        MYCHECK(ISDEBUG);
//        std::cout << probability_.transpose() << std::endl;
//        std::cout << p_state_.transpose() << std::endl;
        if (MethodType == 0) {
            double x(0.0), y(0.0);
            if (std::fabs(this->probability_.sum() - 1.0) > 1e-5) {
                this->probability_ /= this->probability_.sum();
            }
            for (int i(0); i < this->p_state_.rows(); ++i) {
                x += this->probability_(i) * this->p_state_(i, 0);
                y += this->probability_(i) * this->p_state_(i, 1);
            }
            return Eigen::Vector2d(x, y);
        }
    }


    bool OptimateInitial(Eigen::VectorXd state,
                         int MethodType = 0) {
        Eigen::VectorXd last_res, res;
        last_res.resize(this->p_state_.cols());
        res.resize(this->p_state_.cols());
        last_res.setZero();
        res.setZero();
        int times(0);
        while (times < 5 || (res - last_res).norm() > 0.1) {
            std::cout << "TIMES:" << times << std::endl;
            last_res = res;
            StateTransmition(Eigen::Vector2d(0, 0), 0);
            Evaluation(state, 0);
            res = GetResult(0);
            Resample(-1, 0);
            times++;
            if (times > 100) {
                return false;
            }
        }
        std::cout << "result is :" << res.transpose() << std::endl;
        return true;

    }


private:
//    Eigen::MatrixXd p_state_;//particle filter

//    Eigen::VectorXd probability_;//accumulate probability of each particles.


    //Method parameters.

    Eigen::VectorXd input_noise_sigma_;

    Eigen::MatrixXd beacon_set_;

    Eigen::VectorXd measurement_sigma_;


};

#endif //QUICKFUSING_PUWBPF_HPP
