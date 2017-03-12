#pragma once
//Create by steve in 17-1-5 at 上午9:21
//
// Created by steve on 17-1-5.
//

#ifndef QUICKFUSING_EXUWBPF_HPP
#define QUICKFUSING_EXUWBPF_HPP

#include "PFBase.hpp"

//#include "omp.h"


#define ISDEBUG false


template<int uwb_number>
class EXUWBPF : public PFBase<double, 6, uwb_number> {
public:
    EXUWBPF(int particle_num) : PFBase<double, 6, uwb_number>(particle_num) {
        try {
            this->p_state_.resize(particle_num, 6);
            this->p_state_.setZero();
            this->probability_.resize(particle_num);
            this->probability_.setOnes();
            this->probability_ = this->probability_ / this->probability_.sum();

            input_noise_sigma_.resize(this->p_state_.cols());
        } catch (...) {
            MYERROR("EXUWBPF initial error.");
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
        MYCHECK(ISDEBUG);
        return true;
    }


    /**
     * State transmition function,propogate the state along the time sequence.
     *
     * !!! In methodType 2,inpute is delta of velocity and angular not
     * @param input
     * @param MethodType
     * @return
     */
    bool StateTransmition(Eigen::VectorXd input, int MethodType = 0) {
        if (MethodType == 0) {
            MYCHECK(ISDEBUG)
            double sigma = input_noise_sigma_.mean();

            MYCHECK(ISDEBUG)

            std::normal_distribution<double> normal_distribution(0, sigma);
            MYCHECK(ISDEBUG)
#pragma omp parallel for
            for (int i=(0); i < this->p_state_.rows(); ++i) {

                this->p_state_(i, 4) += normal_distribution(this->e_);

                this->p_state_(i, 0) += this->p_state_(i, 2);
                this->p_state_(i, 0) += 0.5 * this->p_state_(i, 4);

                this->p_state_(i, 2) += this->p_state_(i, 4);


                this->p_state_(i, 5) += normal_distribution(this->e_);

                this->p_state_(i, 1) += this->p_state_(i, 3);
                this->p_state_(i, 1) += 0.5 * this->p_state_(i, 5);

                this->p_state_(i, 3) += this->p_state_(i, 5);
            }
            MYCHECK(ISDEBUG)
        } else if (MethodType == 1) {
            double sigma = input_noise_sigma_.mean();

            std::normal_distribution<double> vel_distribution(0, sigma);
            std::normal_distribution<double> ori_distribution(0, sigma / 10 * M_PI);

            /**
             * x,y,theta,v,w,a.
             * w in [-pi,pi].
             * a in [-inf,inf]----([-5,5]);
             *
             */
#pragma omp parallel for
            for (int i=(0); i < this->p_state_.rows(); ++i) {
                //// w
                this->p_state_(i, 4) += ori_distribution(this->e_);

                this->p_state_(i, 5) += vel_distribution(this->e_);

                /////theta v
                this->p_state_(i, 2) += this->p_state_(i, 4);

                this->p_state_(i, 3) += this->p_state_(i, 5);

                ////
                double move(this->p_state_(i, 3) + 0.5 * this->p_state_(i, 5));


                this->p_state_(i, 0) += std::sin(this->p_state_(i, 2)) * move;
                this->p_state_(i, 1) += std::cos(this->p_state_(i, 2)) * move;
            }
        } else if (MethodType == 2) {
            double sigma = input_noise_sigma_.mean();

            std::normal_distribution<double> vel_distribution(input(0), sigma);
            std::normal_distribution<double> ori_distribution(input(1), sigma / 5.0 * M_PI);


            /**
            * x,y,theta,v,w,a.
            * w in [-pi,pi].
            * a in [-inf,inf]----([-5,5]);
            *
            */
#pragma omp parallel for
            for (int i = 0; i < this->p_state_.rows(); ++i) {
                //// w
                this->p_state_(i, 4) = ori_distribution(this->e_);

                this->p_state_(i, 5) = vel_distribution(this->e_);

                /////theta v
                this->p_state_(i, 2) += this->p_state_(i, 4);

                this->p_state_(i, 3) += this->p_state_(i, 5);
//                this->p_state_(i,2) = ori_distribution(this->e_);

//                this->p_state_(i,3) = vel_distribution(this->e_);

                ////
                double move(this->p_state_(i, 3) + 0.5 * this->p_state_(i, 5));


                this->p_state_(i, 0) += std::sin(this->p_state_(i, 2)) * move;
                this->p_state_(i, 1) += std::cos(this->p_state_(i, 2)) * move;

//                if(std::isnan(this->p_state_.sum()))
//                {
//                    std::cout << "ERRORO  in isnan " << std::endl;
//                }else{
//                    std::cout << "Not nana " << std::endl;
//                }
            }

        } else if (MethodType == 3) {

            double sigma = input_noise_sigma_.mean();
            std::normal_distribution<double> state_distribution(input(0), sigma);
#pragma omp parallel for

            for (int i = 0;i<this->p_state_.rows();++i)
            {
                this->p_state_(i,0) += state_distribution(this->e_);
                this->p_state_(i,1) += state_distribution(this->e_);
            }


        }


    }

    bool StateTransmitionWithTimeStep(Eigen::VectorXd input, double time_step) {
        double sigma = input_noise_sigma_.mean();

        std::normal_distribution<double> normal_distribution(0, sigma);
        // TODO: With time step ,need to test on different time step.
        for (int i(0); i < this->p_state_.rows(); ++i) {

            this->p_state_(i, 4) += normal_distribution(this->e_);

            this->p_state_(i, 0) += this->p_state_(i, 2) * time_step;
            this->p_state_(i, 0) += 0.5 * this->p_state_(i, 4) * time_step * time_step;

            this->p_state_(i, 2) += this->p_state_(i, 4) * time_step;


            this->p_state_(i, 5) += normal_distribution(this->e_);

            this->p_state_(i, 1) += this->p_state_(i, 3) * time_step;
            this->p_state_(i, 1) += 0.5 * this->p_state_(i, 5) * time_step * time_step;

            this->p_state_(i, 3) += this->p_state_(i, 5) * time_step;
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
#pragma omp parallel for
            for (int i=(0); i < this->p_state_.rows(); ++i) {
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
            std::cout << "evaluation error " << std::endl;
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
    bool Resample(int resample_num, int MethodType = 0) {
        MYCHECK(ISDEBUG);
        if (MethodType == 0) {

//            std::vector<Eigen::VectorXd> tmp_vec;
//            std::vector<double> tmp_score;
            Eigen::MatrixXd tmp_vec;
            Eigen::VectorXd tmp_score;
            tmp_vec.resizeLike(this->p_state_);
            tmp_score.resizeLike(this->probability_);
            this->probability_ = this->probability_ / this->probability_.sum();


            std::uniform_real_distribution<double> real_distribution(0, 0.9999999);
            MYCHECK(ISDEBUG);

#pragma omp parallel for
            for (int index = 0; index < this->p_state_.rows(); ++index) {
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
//                tmp_vec.push_back(this->p_state_.block(i, 0, 1, this->p_state_.cols()).transpose());
//                MYCHECK(ISDEBUG);
//                tmp_score.push_back(this->probability_(i));
                for (int k(0); k < tmp_vec.cols(); ++k) {
                    tmp_vec(index, k) = this->p_state_(i, k);
                }
                tmp_score(index) = this->probability_(i);
            }
            MYCHECK(ISDEBUG);

//            for (int index(0); index < this->probability_.rows(); ++index) {
//                this->probability_(index) = tmp_score[index];
//                this->p_state_.block(index, 0, 1, this->p_state_.cols()) = tmp_vec[index].transpose();
//            }
            this->probability_ = tmp_score;
            this->p_state_ = tmp_vec;
            if (std::isnan(this->probability_.sum())) {
                this->probability_.setOnes();
            }

//            probability_.setOnes();
            this->probability_ = this->probability_ / this->probability_.sum();
        } else if (MethodType == 1) {

        }
    }

    /**
     * Average with weight.
     * @param MethodType
     * @return
     */
    Eigen::VectorXd GetResult(int MethodType = 0) {
        MYCHECK(ISDEBUG);
//        std::cout << this->probability_.transpose() << std::endl;
//        std::cout << this->p_state_.transpose() << std::endl;
        if (MethodType == 0) {
            double x(0.0), y(0.0);
            if (std::fabs(this->probability_.sum() - 1.0) > 1e-5) {
                this->probability_ /= this->probability_.sum();
            }
            for (int i(0); i < this->p_state_.rows(); ++i) {
                x += this->probability_(i) * this->p_state_(i, 0);
                y += this->probability_(i) * this->p_state_(i, 1);
            }
            MYCHECK(ISDEBUG)
            return Eigen::Vector2d(x, y);
        }
    }

    bool Initial(Eigen::VectorXd State) {
        for (int i(0); i < this->p_state_.rows(); ++i) {
            for (int j(0); j < this->p_state_.cols(); ++j) {
                if (j >= State.rows()) {
                    this->p_state_(i, j) = 0.0;
                } else {
                    this->p_state_(i, j) = State(j);
                }
            }
        }
    }


    bool OptimateInitial(Eigen::VectorXd state,
                         int MethodType = 0) {
        MYCHECK(ISDEBUG);
        Eigen::VectorXd last_res, res;
        MYCHECK(ISDEBUG);
        last_res.resize(this->p_state_.cols());
        res.resize(this->p_state_.cols());
        MYCHECK(ISDEBUG);

        last_res.setZero();
        res.setZero();
        int times(0);
        while (times < 5 || (res - last_res).norm() > 0.1) {
//            std::cout << "Initial TIMES:" << times << std::endl;
            last_res = res;
            StateTransmition(Eigen::Vector2d(0, 0), 0);
            Evaluation(state, 0);
            res = GetResult(0);
            std::cout << "res :" << res.transpose() << std::endl;
            Resample(-1, 0);
            times++;
            if (times > 100) {
                return false;
            }
        }
        std::cout << "result is :" << res.transpose() << std::endl;
        return true;

    }

protected:


private:
    Eigen::VectorXd input_noise_sigma_;

    Eigen::MatrixXd beacon_set_;

    Eigen::VectorXd measurement_sigma_;

};

#endif //QUICKFUSING_EXUWBPF_HPP
