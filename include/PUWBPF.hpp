#pragma once
//Create by steve in 16-12-4 at 下午6:12
//
// Created by steve on 16-12-4.
//

#include "PFBase.hpp"

#ifndef QUICKFUSING_PUWBPF_HPP
#define QUICKFUSING_PUWBPF_HPP

#define ISDEBUG true

template<int uwb_number>
class PUWBPF : public PFBase<double, 2, uwb_number> {
public:
    PUWBPF(int particle_num) : PFBase<double, 2, uwb_number>(particle_num) {
//        PFBase(particle_num);
        try {
            p_state_.resize(particle_num, 2);
            p_state_.setZero();
            input_noise_sigma_.resize(p_state_.cols());
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
    }

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
        beacon_set_ = beacon_set_;
        return true;
    }

    /*
     * State transmission equation.
     */
    bool StateTransmition(Eigen::VectorXd input, int MethodType = 0) {
        MYCHECK(ISDEBUG);
        if (MethodType == 0)//Method 0:Random move follow the Gaussian distribution(Same sigma).
        {
            double sigma = input_noise_sigma_.mean();
            MYCHECK(ISDEBUG);
            std::default_random_engine ee_;
            std::normal_distribution<double> normal_distribution(0, sigma);
            MYCHECK(ISDEBUG);
            for (int i(0); i < p_state_.rows(); ++i) {
                for (int j(0); j < p_state_.cols(); ++j) {
                    p_state_(i, j) += normal_distribution(ee_);
                    MYCHECK(ISDEBUG);
                }
            }
            return true;
        }
    }

    /*
     * Evaluation function.
     * Input state and measurement data,and compute a score.
     */
    bool Evaluation(Eigen::VectorXd measurement, int MethodType = 0) {
        MYCHECK(ISDEBUG);
        if (MethodType == 0) {
            for (int i(0); i < p_state_.rows(); ++i) {
                probability_(i) = EvaluationSingle(p_state_.block(i, 0, 1, p_state_.cols()),
                                                   measurement);
            }
        }
        probability_ /= probability_.sum();
        return true;

    }

    double EvaluationSingle(Eigen::VectorXd state,
                            Eigen::VectorXd measurement) {
        MYCHECK(ISDEBUG);
        double score(0.0);
        try {
            for (int i(0); i < beacon_set_.rows(); ++i) {
                double dis(0.0);
                for (int j(0); j < beacon_set_.cols(); ++j) {
                    dis += std::pow(state(j) - beacon_set_(i, j), 2.0);
                }
                dis = std::sqrt(dis);
                score += this->ScalarNormalPdf(dis, measurement(i), measurement_sigma_(i));
            }
        } catch (...) {
            return 0.0;
        }


        return score;
    }

    /*
     * Resample
     *
     * MethodType:
     * 0: Typical resample method.resample_num is not used in this method.
     * 1: Layer-based resample method.
     */
    bool Resample(int resample_num, int MethodType = 0) {
        MYCHECK(ISDEBUG);
        if (MethodType == 0) {

            std::vector<Eigen::VectorXd> tmp_vec;
            std::vector<double> tmp_score;

            std::uniform_real_distribution<double> real_distribution(0, 0.9999999);

            for (int index(0); index < p_state_.rows(); ++index) {
                double score = real_distribution(this->e_);

                int i(0);
                while (score > 0.0) {
                    score -= probability_(i);
                    ++i;
                }
                tmp_vec.push_back(p_state_.block(i, 0, 1, p_state_.cols()));
                tmp_score.push_back(probability_(i));

            }

            for (int index(0); index < probability_.rows(); ++index) {
                probability_(index) = tmp_score[index];
                p_state_.block(index, 0, 1, p_state_.cols()) = tmp_vec[index];
            }
        }
    }


    Eigen::VectorXd GetResult(int MethodType = 0) {
        MYCHECK(ISDEBUG);
        if (MethodType == 0) {
            double x(0.0), y(0.0);
            if (std::fabs(probability_.sum() - 1.0) > 1e-5) {
                probability_ /= probability_.sum();
            }
            for (int i(0); i < p_state_.rows(); ++i) {
                x += probability_(i) * p_state_(i, 0);
                y += probability_(i) * p_state_(i, 1);
            }
            return Eigen::Vector2d(x, y);
        }
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
