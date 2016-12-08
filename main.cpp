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

#include "ResultEvaluation.hpp"

#include "RangeKF.hpp"

#include "PUWBPF.hpp"

/////stamp---------


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


int main(int argc, char *argv[]) {


    /*
     * Load Imu data.
     */
    std::string dir_name = "tmp_file_dir/";

    CSVReader ImuDataReader(dir_name + "ImuData.data.csv"), ZuptReader(dir_name + "Zupt.data.csv");

    auto ImuDataTmp(ImuDataReader.GetMatrix()), ZuptTmp(ZuptReader.GetMatrix());

    Eigen::MatrixXd ImuData, Zupt;
    ImuData.resize(ImuDataTmp.GetRows(), ImuDataTmp.GetCols());
    Zupt.resize(ZuptTmp.GetRows(), ZuptTmp.GetCols());

    ////////////ADD NOISE TO SOURCE DATA
    std::default_random_engine ee;
    std::uniform_real_distribution<double> u(-0.15,0.15);
    std::normal_distribution<> n(0.0,0.2);

    for (int i(0); i < ImuDataTmp.GetRows(); ++i) {
        for (int j(0); j < ImuDataTmp.GetCols(); ++j) {
            ImuData(i, j) = *ImuDataTmp(i, j)  ;
        }
        Zupt(i, 0) = *ZuptTmp(i, 0);
    }


    /*
     * Set initial parameter for ekf.
     */
    SettingPara init_para(true);

    init_para.init_pos1_ = Eigen::Vector3d(0.8, -5.6, 0.0);
    init_para.init_heading1_ = -180 / 180 * M_PI;
//    init_para.init_heading1_ = 0.0 + 20 / 180.0 *M_PI;
    init_para.Ts_ = 1.0 / 128.0;

//    init_para.sigma_gyro_ *= 1.3;

    /*
     * Initial filter.
     */

    double b_time(TimeStamp::now());


    Ekf ekf(init_para);

    ekf.InitNavEq(ImuData.block(0, 1, 20, 6));

    std::vector<double> imux, imuy;
    for (int i(0); i < ImuData.rows(); ++i) {

        Eigen::VectorXd vec = ekf.GetPosition(ImuData.block(i, 1, 1, 6).transpose(), Zupt(i));
        if (std::isnan(vec(0))) {
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
    std::vector<std::vector<double>> range_vec;



    //////////////-------------------UWB FILTER------------
    for(int i(0);i<UwbData.cols();++i)
    {
        range_vec.push_back(std::vector<double>());
        SingleValueFilter sf(0.4,0.4);
        for(int j(0);j<UwbData.rows();++j)
        {
            UwbData(j,i) = sf.filter(UwbData(j,i));
            range_vec[i].push_back(double(UwbData(j,i)));
        }
    }


    /////////////-----------------Load UWB RESULT --------------------
    CSVReader UwbresultReader(dir_name + "UwbResult.data.csv");

    std::vector<double> ux, uy;
//    for (int i(0); i < UwbresultReader.GetMatrix().GetRows(); ++i) {
//        ux.push_back(*UwbdataReader.GetMatrix()(i, 1));
//        uy.push_back(*UwbdataReader.GetMatrix()(i, 2));
//    }
    /////////////////////---Compute result only uwb data.
    PUWBPF<4> puwbpf(1000);

    puwbpf.SetMeasurementSigma(1.0,4);
    puwbpf.SetInputNoiseSigma(1.0);

    puwbpf.SetBeaconSet(beaconset);

    for(int i(0);i<UwbData.rows();++i)
    {
        puwbpf.StateTransmition(Eigen::Vector2d(2,2),0);

        puwbpf.Evaluation(UwbData.block(i,1,1,UwbData.cols()-1),0);

        puwbpf.Resample(-1,0);
    }






//   std::cout << beaconset << std::endl;
//
//    std::cout <<"  uwbdata :" << std::endl << UwbData << std::endl;

    /*
     * Particle filter
     */


    //-----------  TIME OFFSET______
//    if(UwbData(0,0) - ImuData(0,0) > 100)
//    {
////        ImuData.block(0,0,ImuData.rows(),1) = ImuData.block(0,0,ImuData.rows(),1) +
////                531844067.535;531844066.53
//        for(int k(0);k<ImuData.rows();++k)
//        {
//            ImuData(k,0) = ImuData(k,0) + 531844066.53;
//        }
//    }

    /////-------------Filter parameter----------------------

    int particle_num = 8000;
    double noise_sigma = 2.0;
    double evaluate_sigma = 2.6;
    double filter_btime(TimeStamp::now());

    if (argc != 4) {
        std::cout << "output is zero" << std::endl;
    } else {
        particle_num = atoi(argv[1]);
        noise_sigma = atof(argv[2]);
        evaluate_sigma = atof(argv[3]);
    }

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
                    if(j<3)
                    {
                        noise(j) = n_distribution(e) * init_para.sigma_acc_(j);
                    }else{
                        noise(j) = n_distribution(e) * init_para.sigma_gyro_(j-3);
                    }

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


    //////////////////////----------------COMPUTE ERROR---------------------////
    std::vector<double> imu_err,fusing_err;
    double avg_imu(0.0),avg_fusing(0.0);

    ResultEvaluation re(dir_name + "keypoint.csv");

    //imu
    for(int i(0);i<imux.size();++i)
    {
        imu_err.push_back(re.Distance(
                Eigen::Vector2d(imux[i],imuy[i]),
                ImuData(i,0)));

    }
    avg_imu = std::accumulate(imu_err.begin(),imu_err.end(),0.0);
    avg_imu /= double(imu_err.size());

    //fusing
    for(int i(0);i<fx.size();++i)
    {
        fusing_err.push_back(
                re.Distance(
                        Eigen::Vector2d(fx[i],fy[i]),
                        UwbData(i,0)
                )
        );
    }
    avg_fusing = std::accumulate(fusing_err.begin(),fusing_err.end(),0.0);
    avg_fusing /= double(fusing_err.size());



    std::cout << "avg_error of imu:" << avg_imu << " avg_error of fusing: " << avg_fusing << std::endl;




//                plt::show();
    ////////////////////////////////Show result /////////////////////////////////
    plt::subplot(2,2,0);
    plt::named_plot("Imu result", imux, imuy, "r.");
    plt::named_plot("Fusing result", fx, fy, "b+-");
    plt::named_plot("real path", rx, ry, "g-");
//    plt::named_plot("Uwb Result",ux,uy,"y+-");
////    plt::legend();
    plt::title(std::to_string(particle_num)
               +"-"+std::to_string(noise_sigma) +"-"
               + std::to_string(evaluate_sigma) +"-"
               +"avgimu"+std::to_string(avg_imu) + "-"
               +"avgfus"+std::to_string(avg_fusing) + "-"
               +std::to_string(TimeStamp::now()));
    plt::grid(true);

    plt::subplot(2,2,1);
    plt::grid(true);



    plt::save(std::to_string(particle_num)
              + "-" + std::to_string(noise_sigma) + "-"
              + std::to_string(evaluate_sigma) + "-"
              + std::to_string(TimeStamp::now()) + ".jpg");

//    plt::show();

    return 0;


}
