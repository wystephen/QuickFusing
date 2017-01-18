//Create by steve in 16-12-31 at 上午9:34
//
// Created by steve on 16-12-31.
//

#include <iostream>
#include <fstream>
#include <cmath>
#include <random>

#include <omp.h>

#include <eigen3/Eigen/Dense>
#include <ImuIntegrate.h>

#include "CSVReader.h"
#include "matplotlib_interface.h"
#include "time_stamp.h"

#include "SettingPara.h"
#include "EKF.hpp"

#include "ResultEvaluation.hpp"

/////stamp---------

#include "RangeKF.hpp"

#include "PUWBPF.hpp"

#include "EXUWBPF.hpp"


#include "MYEKF.h"

/////stamp---------
namespace plt = matplotlibcpp;

/*
 * return the probability of N(x|miu,sigma);
 */
double NormalPdf(double x,
                 double miu,
                 double sigma) {
//    std::cout << "dis :" << x << " range:" << miu << std::endl;
    double para1((x - miu) * (x - miu) / 2 / sigma / sigma);
    double para2(1 / std::sqrt(2 * sigma * sigma * M_PI));
    return para2 * std::exp(-para1);
}

/*
 * Evaluation function.
 */
double Pdf(Eigen::Vector2d vecx,
           Eigen::MatrixXd beaconset,
           Eigen::VectorXd Range,
           double z_offset, double sigma) {
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
        std::cout << "ERROR When output likelihood probability distribution function."
                  << std::endl;
        return 1.0;
    }

}


int main(int argc, char *argv[]) {

    std::cout.precision(20); //

    double first_t(TimeStamp::now());

    /*
     * Load Imu data.
     */

    std::string dir_name = "tmp_file_dir---/";

    CSVReader ImuDataReader(dir_name + "ImuData.data.csv"),
            ZuptReader(dir_name + "Zupt.data.csv");

    auto ImuDataTmp(ImuDataReader.GetMatrix()), ZuptTmp(ZuptReader.GetMatrix());

    Eigen::MatrixXd ImuData, Zupt;
    ImuData.resize(ImuDataTmp.GetRows(), ImuDataTmp.GetCols());
    Zupt.resize(ZuptTmp.GetRows(), ZuptTmp.GetCols());

    //////////ADD NOISE TO SOURCE DATA
    std::default_random_engine ee;
    std::uniform_real_distribution<double> u(-0.15, 0.15);
    std::normal_distribution<> n(0.0, 0.2);

    for (int i(0); i < ImuDataTmp.GetRows(); ++i) {
        for (int j(0); j < ImuDataTmp.GetCols(); ++j) {
            ImuData(i, j) = *ImuDataTmp(i, j);
        }
        Zupt(i, 0) = int(*ZuptTmp(i, 0));
//        std::cout << i << " :  " << *ZuptTmp(i,0) << std::endl;
    }


    /**
     * ImuIntegrate
     */

    std::vector<double> ix, iy;

    ImuIntegrate imuinteg(1.0);

    for (int i(1); i < ImuData.rows() / 100.0 * 3; ++i) {
        Eigen::VectorXd tmp;
        tmp = imuinteg.IntegratingState(ImuData.block(i, 1, 1, ImuData.cols() - 2).transpose(),
                                        1.0 / 128.0);
//                                        1.0);
        std::cout << " imu integrate : "
                  << i << "  :  "
                  << tmp.transpose() << std::endl;
//        ix.push_back(double(tmp(0)));
//        iy.push_back(double(tmp(1)));
    }

    std::cout << "total time:" << ImuData.rows() / 100.0 * 3 / 128.0
              << std::endl;



    /**
     * MyEkf
     */

    std::vector<double> mx, my;

    SettingPara init_para(true);

    init_para.init_pos1_ = Eigen::Vector3d(1.45, -6.3, 0.0);
    init_para.init_heading1_ = 0.0 + 20 / 180.0 * M_PI;

    init_para.Ts_ = 1.0 / 128.0;

    MyEkf myekf(init_para);

    myekf.InitNavEq(ImuData.block(0, 1, 20, 6));


    std::cout << "IMU rows :" << ImuData.rows() << std::endl;

    std::vector<double> wi, w1, w2, w3;

    for (int i(0); i < ImuData.rows(); ++i) {
        Eigen::VectorXd vec = myekf.GetPosition(
                ImuData.block(i, 1, 1, 6).transpose(),
                Zupt(i, 0));

//        std::cout << Zupt(i) << std::endl;
//        std::cout << vec.transpose() << std::endl;
//        myekf.ComputeHeading();

        wi.push_back(double(i));
//        w1.push_back(myekf.getOriente());
//        w2.push_back(myekf.getVelocity());
//        std::cout << myekf.getVelocity() << std::endl;
//        w2.push_back(vec(4));
//        w3.push_back(vec(5));



        mx.push_back(double(vec(0)));
        my.push_back(double(vec(1)));
    }

    std::cout << " zupt sum : " << Zupt.sum() << " size : " << Zupt.size()
              << std::endl;







    /**
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


    //Fusing result.
    std::vector<double> fx, fy;
    std::vector<double> ux, uy;

    std::cout << TimeStamp::now() - first_t << std::endl;


    /**
     * PF with only uwb.
     */
//    PUWBPF<4> puwbpf(1000);
    EXUWBPF<4> puwbpf(12100);


    puwbpf.SetMeasurementSigma(5.0, 4);
    puwbpf.SetInputNoiseSigma(0.20);

    puwbpf.SetBeaconSet(beaconset);
    std::cout << "result:" << puwbpf.GetResult(0) << std::endl;

    puwbpf.OptimateInitial(UwbData.block(10, 1, 1, UwbData.cols() - 1).transpose(), 0);


    for (int i(0); i < UwbData.rows(); ++i) {
//        std::cout << "1.1\n";
        if ((i / UwbData.rows() * 100) % 10 == 0) {
//            std::cout << "finished :" << double(i) / double(UwbData.rows()) * 100.0 << "  %.\n";
        }

        puwbpf.StateTransmition(Eigen::Vector2d(2, 2), 1);

        puwbpf.Evaluation(UwbData.block(i, 1, 1, UwbData.cols() - 1).transpose(),
                          0);
        Eigen::VectorXd tmp = puwbpf.GetResult(0);

        puwbpf.Resample(-1, 0);

        ux.push_back(double(tmp(0)));
        uy.push_back(double(tmp(1)));
    }

    /**
     * Fusing....
     */

    int uwb_index(0), imu_index(0);

    double last_v(0),last_ori(0);

    EXUWBPF<4> muwbpf(31000);
    muwbpf.SetMeasurementSigma(3.0, 4);
    muwbpf.SetInputNoiseSigma(0.20);
    muwbpf.SetBeaconSet(beaconset);
//    std::cout << "herererererere" << std::endl;
//    std::cout <<  UwbData.block(10,1,1,UwbData.cols()-1) << std::endl;
    muwbpf.OptimateInitial(UwbData.block(10, 1, 1, UwbData.cols() - 1).transpose(), 0);

    MyEkf mixekf(init_para);
    mixekf.InitNavEq(ImuData.block(0, 1, 20, 6));

    double fusing_start_time(TimeStamp::now());

    while (true) {
        if (uwb_index >= UwbData.rows() || imu_index >= ImuData.rows()) {

            break;
        }

        if (UwbData(uwb_index, 0) < ImuData(imu_index, 0)) {
            /*
             * update Uwb data
             */

//            std::cout << "ekf velocity :"
//                      << mixekf.getVelocity()
//                      << " ori : "
//                      << mixekf.getOriente() << std::endl;

            double delta_ori(mixekf.getOriente()-last_ori);
            delta_ori  = delta_ori / 180.0 * M_PI;

            if(delta_ori > M_PI)
            {
                delta_ori -= (2*M_PI);
            }else if(delta_ori < -M_PI){
                delta_ori += (2.0 * M_PI);
            }

            muwbpf.StateTransmition(Eigen::Vector2d((mixekf.getVelocity()-last_v) ,
                                                    delta_ori//(mixekf.getOriente()-last_ori )/ 180.0 * M_PI
                                                    ),
                                    2);

            w1.push_back(mixekf.getVelocity()-last_v);// red
//            w2.push_back((mixekf.getOriente()-last_ori )/ 180.0 * M_PI);//green
            w2.push_back(delta_ori);

            last_v = mixekf.getVelocity();
            last_ori = mixekf.getOriente();

            muwbpf.Evaluation(UwbData.block(uwb_index, 1, 1, UwbData.cols() - 1).transpose(),
                              0);

            Eigen::VectorXd tmp = muwbpf.GetResult(0);
            muwbpf.Resample(-1, 0);

            fx.push_back(double(tmp(0)));
            fy.push_back(double(tmp(1)));
            uwb_index++;
        } else {
            /*
             * update imu data
             */
            mixekf.GetPosition(ImuData.block(imu_index, 1, 1, 6).transpose(),
                               Zupt(imu_index, 0));

            imu_index++;
        }
    }

    std::cout << "fusing used time:" << TimeStamp::now() - fusing_start_time
              << "data total time :" << UwbData(UwbData.rows()-1,0)-UwbData(0,0)
              << std::endl;

    /**
     * Show result.
     */
    plt::named_plot("u", ux, uy, "r-+");
    plt::named_plot("i", ix, iy, "b-+");
    plt::named_plot("m", mx, my, "y-+");
    plt::named_plot("f", fx, fy, "g-+");
    plt::plot(w1,"r-+");
    plt::plot(w2,"g-+");
//    plt::plot(w3,"b-+");
    plt::legend();

//    plt::named_plot("ux1", ux, ux);
    plt::grid(true);
    plt::show();


}

