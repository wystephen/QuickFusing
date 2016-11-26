#include <iostream>
#include <fstream>
#include <cmath>
#include <random>

#include <omp.h>

#include <eigen3/Eigen/Dense>

#include "CSVReader.h"
#include "matplotlib_interface.h"
#include "time_stamp.h"

#include "SettingPara.h"
#include "EKF.hpp"


namespace plt = matplotlibcpp;

/*
 * return the probability of N(x|miu,sigma);
 */
double NormalPdf(double x, double miu, double sigma) {
//    std::cout << "dis :" << x << " range:" << miu << std::endl;
    double para1((x - miu) * (x - miu) / 2 / sigma / sigma);
    double para2(1 / std::sqrt(2 * sigma * sigma * M_PI));
    return para2 * std::exp(-para1);
}

/*
 * Evaluation function.
 */
double Pdf(Eigen::Vector2d vecx, Eigen::MatrixXd beaconset, Eigen::VectorXd Range, double z_offset, double sigma) {
    try {
        double res(1.0);
        double dis(0.0);
        for (int i(0); i < beaconset.rows(); ++i) {
            dis = 0.0;
            dis += (vecx(0) - beaconset(i, 0)) * (vecx(0) - beaconset(i, 0));
            dis += (vecx(1) - beaconset(i, 1)) * (vecx(1) - beaconset(i, 1));
            dis += (z_offset - beaconset(i, 2)) * (z_offset - beaconset(i, 2));

            res += NormalPdf(Range(i), std::sqrt(dis), sigma);
        }
        return res;
    } catch (...) {
        std::cout << "ERROR When output likelihood probability distribution function." << std::endl;
        return 1.0;
    }

}


int main(int argc,char * argv[]) {


    /*
     * Load Imu data.
     */
    std::string dir_name = "tmp_file_dir/";

    CSVReader ImuDataReader(dir_name + "ImuData.data.csv"), ZuptReader(dir_name + "Zupt.data.csv");

    auto ImuDataTmp(ImuDataReader.GetMatrix()), ZuptTmp(ZuptReader.GetMatrix());

    Eigen::MatrixXd ImuData, Zupt;
    ImuData.resize(ImuDataTmp.GetRows(), ImuDataTmp.GetCols());
    Zupt.resize(ZuptTmp.GetRows(), ZuptTmp.GetCols());

    for (int i(0); i < ImuDataTmp.GetRows(); ++i) {
        for (int j(0); j < ImuDataTmp.GetCols(); ++j) {
            ImuData(i, j) = *ImuDataTmp(i, j);
        }
        Zupt(i, 0) = *ZuptTmp(i, 0);
    }


    /*
     * Set initial parameter for ekf.
     */
    SettingPara init_para(true);

    init_para.init_pos1_ = Eigen::Vector3d(0.8, -5.6, 0.0);
    init_para.init_heading1_ = -180 / 180 * M_PI;

    init_para.Ts_ = 1.0 / 128.0;

    /*
     * Initial filter.
     */

    double b_time(TimeStamp::now());


    Ekf ekf(init_para);

    ekf.InitNavEq(ImuData.block(0, 1, 20, 6));

    std::vector<double> imux, imuy;
    for (int i(0); i < ImuData.rows(); ++i) {

        Eigen::VectorXd vec = ekf.GetPosition(ImuData.block(i, 1, 1, 6).transpose(), Zupt(i));
        if (isnan(vec(0))) {
            std::cout << "ddd" << std::endl;
            MYERROR("value change to nan")
            break;

        } else {
            imux.push_back(double(vec(0)));
            imuy.push_back(double(vec(1)));
        }
//        std::cout << i << ":" << ImuData.rows() << ":" << Zupt(i) << "   :   " << vec.transpose() << std::endl;
    }
//    std::cout << " Cost time :" << TimeStamp::now() - b_time << std::endl;



    /*
     * Load uwb data.
     */

    CSVReader BeaconsetReader(dir_name + "beaconset.data.csv");
    CSVReader UwbdataReader(dir_name + "UwbData.data.csv");

    Eigen::MatrixXd beaconset, UwbData;

    beaconset.resize(BeaconsetReader.GetMatrix().GetRows(), BeaconsetReader.GetMatrix().GetCols());
    for (int i(0); i < beaconset.rows(); ++i) {
        for (int j(0); j < beaconset.cols(); ++j) {
            beaconset(i, j) = *BeaconsetReader.GetMatrix()(i, j);
        }
    }

    UwbData.resize(UwbdataReader.GetMatrix().GetRows(), UwbdataReader.GetMatrix().GetCols());
    for (int i(0); i < UwbData.rows(); ++i) {
        for (int j(0); j < UwbData.cols(); ++j) {
            UwbData(i, j) = *(UwbdataReader.GetMatrix()(i, j));
        }
    }

    /////////////-----------------Load UWB RESULT --------------------
    CSVReader UwbresultReader(dir_name + "UwbResult.data.csv");

    std::vector<double> ux, uy;
    for (int i(0); i < UwbresultReader.GetMatrix().GetRows(); ++i) {
        ux.push_back(*UwbdataReader.GetMatrix()(i, 1));
        uy.push_back(*UwbdataReader.GetMatrix()(i, 2));
    }


//   std::cout << beaconset << std::endl;
//
//    std::cout <<"  uwbdata :" << std::endl << UwbData << std::endl;

    /*
     * Particle filter
     */

    /////-------------Filter parameter----------------------

    int particle_num = 400;
    double noise_sigma = 0.1;
    double evaluate_sigma = 1.0;
    double filter_btime(TimeStamp::now());

    if(argc == 0)
    {
        std::cout << "output is zero"<< std::endl;
    }else{
        particle_num = atoi(argv[1]);
        noise_sigma = atof(argv[2]);
        evaluate_sigma = atof(argv[3]);
    }

//    std::vector<int> particle_number_vec;
//    particle_number_vec.push_back(40);
//    particle_number_vec.push_back(60);
//    particle_number_vec.push_back(200);
//    particle_number_vec.push_back(2000);
//    particle_number_vec.push_back(4000);
//    particle_number_vec.push_back(10000);
//    particle_number_vec.push_back(20000);
//    particle_number_vec.push_back(6000);
//
//
//    std::vector<double> noise_sigam_vec;
//    noise_sigam_vec.push_back(0.01);
//    noise_sigam_vec.push_back(0.04);
//    noise_sigam_vec.push_back(0.1);
//    noise_sigam_vec.push_back(0.4);
//    noise_sigam_vec.push_back(1.0);
//    noise_sigam_vec.push_back(1.2);
//    noise_sigam_vec.push_back(1.6);
//    noise_sigam_vec.push_back(2.2);
//
//    std::vector<double> evaluate_sigam_vec;
//    evaluate_sigam_vec.push_back(1.0);
//    evaluate_sigam_vec.push_back(3.0);
//    evaluate_sigam_vec.push_back(6.0);
//    evaluate_sigam_vec.push_back(0.5);
//    evaluate_sigam_vec.push_back(2.5);
//
//    for (int test_x(0); test_x < particle_number_vec.size(); ++test_x) {
//        for (int test_y(0); test_y < noise_sigam_vec.size(); ++test_y) {
//            for (int test_z(0); test_z < evaluate_sigam_vec.size(); ++test_z) {
//                particle_num = particle_number_vec[test_x];
//                noise_sigma = noise_sigam_vec[test_y];
//                evaluate_sigma = evaluate_sigam_vec[test_z];
//

                ///////---------------Save result----------------

                std::vector<double> fx, fy;


                /*
                 * Random engine and normal distribution
                 */
                std::default_random_engine e;
                std::normal_distribution<> n_distribution(0, noise_sigma);


                std::vector<Ekf> P_vec;
                std::vector<Eigen::Vector2d> Pose_vec;
                std::vector<double> Score_vec;

                Ekf ekf_sample(init_para);
                ekf_sample.InitNavEq(ImuData.block(0, 1, 20, 6));

                for (int i(0); i < particle_num; ++i) {
                    P_vec.push_back(Ekf(ekf_sample));

                    Pose_vec.push_back((Eigen::Vector2d(0.0, 0.0)));

                    Score_vec.push_back(1.0);
                }

                int imu_step(0), uwb_step(0);

                while (true) {
                    if (imu_step == ImuData.rows() || uwb_step == UwbData.rows()) {
                        break;
                    }

                    if (imu_step == 0) {
                        Eigen::VectorXd tx;
                        for (int i(0); i < P_vec.size(); ++i) {
//                std::cout << " i-u : "<< i << std::endl;
                            Pose_vec[i] = (P_vec[i].GetPosition(ImuData.block(imu_step, 1, 1, 6).transpose(),
                                                                Zupt(imu_step))).block(0, 0, 2, 1);
                        }
                        ++imu_step;
                    }
                    if (uwb_step == 0) {
                        ++uwb_step;
                    }

                    if (ImuData(imu_step, 0) < UwbData(uwb_step, 0)) {
                        ////////---------------------SAMPLE--------------------------///////////////
//            Eigen::VectorXd tx;


#pragma omp parallel for
                        for (int i = 0; i < P_vec.size(); ++i) {
                            Eigen::VectorXd noise;
                            noise.resize(6);
                            for (int j(0); j < 6; ++j) {
                                noise(j) = n_distribution(e);
                            }
                            Pose_vec[i] = (P_vec[i].GetPosition(ImuData.block(imu_step, 1, 1, 6).transpose() + noise,
                                                                Zupt(imu_step))).block(0, 0, 2, 1);
                        }
                        ++imu_step;
                    } else {
                        ////////-------------------------EVALUATE-------------------/////////////////

                        double sum_score(0.0);

                        for (int i(0); i < Score_vec.size(); ++i) {
                            Score_vec[i] *= Pdf(Pose_vec[i],
                                                beaconset,
                                                UwbData.block(uwb_step, 1, 1, UwbData.cols() - 1).transpose(),
                                                1.95,
                                                evaluate_sigma);
                            sum_score += Score_vec[i];
                        }
#pragma omp parallel for
                        for (int i = (0); i < Score_vec.size(); ++i) {
                            Score_vec[i] = Score_vec[i] / sum_score;
                        }



                        ////////////-----------------RESAMPLE---------------------/////////////////////////

                        std::vector<Ekf> tmp_p = P_vec;
//            P_vec.clear();
                        std::vector<double> tmp_score = Score_vec;
//            Score_vec.clear();

                        std::uniform_real_distribution<> uniform_distribution(0.0, 0.999999);
                        if (uwb_step % 20 == 0) {
                            std::cout << "Finished :" << double(uwb_step) / double(UwbData.rows()) * 100.0 << "  % "
                                      << std::endl;
                        }
//            std::cout << "uwb index: " << uwb_step << std::endl;

#pragma omp parallel for
                        for (int i = 0; i < tmp_p.size(); ++i) {
                            double val(uniform_distribution(e));

                            int target_index(0);
                            while (val > 0.0) {
                                val -= tmp_score[target_index];
                                ++target_index;
                                if (target_index >= tmp_p.size()) {
                                    target_index = tmp_p.size() - 1;
                                }
                            }

                            P_vec[i] = (Ekf(tmp_p[target_index]));
                            Score_vec[i] = (tmp_score[target_index]);

                        }

                        ///////-------------------GET RESULT-------------------////////////////////
                        sum_score = 0.0;
                        for (int i(0); i < Score_vec.size(); ++i) {
                            sum_score += Score_vec[i];

                        }
//#pragma omp parallel for
                        for (int i = (0); i < Score_vec.size(); ++i) {
                            Score_vec[i] = Score_vec[i] / sum_score;
                        }

                        double tmp_x(0.0), tmp_y(0.0);
//#pragma omp parallel for
                        for (int i = 0; i < Score_vec.size(); ++i) {
                            tmp_x += Score_vec[i] * Pose_vec[i](0);
                            tmp_y += Score_vec[i] * Pose_vec[i](1);
                        }

                        fx.push_back(tmp_x);
                        fy.push_back(tmp_y);


                        ++uwb_step;
                    }

                }

                std::cout << " Filter total time is : " << TimeStamp::now() - filter_btime << std::endl;
                std::cout << " Data total time is :" << UwbData(UwbData.rows() - 1, 0) - UwbData(0, 0) << std::endl;


                std::cout << " Uwb data :" << UwbData.rows() << "  :   " << UwbData.cols() << std::endl;

                std::cout << "fx fy :" << fx.size() << "  :  " << fy.size() << std::endl;



                ////////////////////------------------------------------------------//////////////////////////////

                ///////////////////////////*Real path*///////////////////////////////////////////////////////
                std::vector<double> rx, ry;
                rx.push_back(0.8);
                ry.push_back(-5.6);

                rx.push_back(-8.0);
                ry.push_back(-5.6);

                rx.push_back(-8.0);
                ry.push_back(-2.4);

                rx.push_back(0.8);
                ry.push_back(-2.4);

                rx.push_back(0.8);
                ry.push_back(2.4);

                rx.push_back(-8.0);
                ry.push_back(2.4);

                rx.push_back(-8.0);
                ry.push_back(5.6);

                rx.push_back(0.8);
                ry.push_back(5.6);

                rx.push_back(0.8);
                ry.push_back(-5.6);


//                plt::show();
                ////////////////////////////////Show result /////////////////////////////////
                plt::named_plot("Imu result", imux, imuy, "r.");
                plt::named_plot("Fusing result", fx, fy, "b+-");
                plt::named_plot("real path", rx, ry, "g-");
//    plt::named_plot("Uwb Result",ux,uy,"y+-");
////    plt::legend();
//    plt::title(std::to_string(particle_num)
//               +"-"+std::to_string(noise_sigma) +"-"
//               + std::to_string(evaluate_sigma) +"-"
//               +std::to_string(TimeStamp::now()));
                plt::grid(true);
                plt::save("dir_name-" + std::to_string(particle_num)
                          + "-" + std::to_string(noise_sigma) + "-"
                          + std::to_string(evaluate_sigma) + "-"
                          + std::to_string(TimeStamp::now()) + ".jpg");

//    plt::show();

//            }
//
//        }
//    }
    return 0;


}