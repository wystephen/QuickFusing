/** 
                   _ooOoo_ 
                  o8888888o 
                  88" . "88 
                  (| -_- |) 
                  O\  =  /O 
               ____/`---'\____ 
             .'  \\|     |//  `. 
            /  \\|||  :  |||//  \ 
           /  _||||| -:- |||||-  \ 
           |   | \\\  -  /// |   | 
           | \_|  ''\---/''  |   | 
           \  .-\__  `-`  ___/-. / 
         ___`. .'  /--.--\  `. . __ 
      ."" '<  `.___\_<|>_/___.'  >'"". 
     | | :  `- \`.;`\ _ /`;.`/ - ` : | | 
     \  \ `-.   \_ __\ /__ _/   .-` /  / 
======`-.____`-.___\_____/___.-`____.-'====== 
                   `=---=' 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
         佛祖保佑       永无BUG 
*/
//
// Created by steve on 17-11-3.
//

#ifndef QUICKFUSING_RANGEEKF_H
#define QUICKFUSING_RANGEEKF_H

#include <MYEKF.h>

class RangeEKF : public MyEkf {
public:


    RangeEKF(SettingPara para) :
            MyEkf(para) {
        std::cout << "...... RangeEKF initialized" << std::endl;

    }

    Eigen::Matrix<double, 1, 9> G;
    Eigen::Matrix<double, 1, 1> GR;
    Eigen::Matrix<double,9,9> tP_;

    bool CorrectRange(Eigen::Vector3d beacon_position,
                      double range_val, double range_sigma) {
        bool JustDebugging = false;
        tP_.setOnes();
        tP_ *= 0.1;

        if (JustDebugging) std::cout << __FILE__ << ":" << __LINE__ << std::endl;
        GR.setZero();
        if (JustDebugging) std::cout << __FILE__ << ":" << __LINE__ << std::endl;
        GR(0, 0) = range_sigma;
        if (JustDebugging) std::cout << __FILE__ << ":" << __LINE__ << std::endl;
        G.setZero();
        if (JustDebugging) std::cout << __FILE__ << ":" << __LINE__ << std::endl;
        Eigen::Matrix<double, 1, 1> gfunc_val;
        gfunc_val.setZero();

        if (JustDebugging) std::cout << __FILE__ << ":" << __LINE__ << std::endl;
        double distance = (beacon_position - x_h_.block(0, 0, 3, 1)).norm();

        if (JustDebugging) std::cout << __FILE__ << ":" << __LINE__ << std::endl;

        for (int i(0); i < 3; ++i) {
            if (JustDebugging) std::cout << __FILE__ << ":" << __LINE__ << std::endl;
            G(i) = -(x_h_(i) - beacon_position(i)) / distance;

//            gfunc_val(i,0)
        }
        if (JustDebugging) std::cout << __FILE__ << ":" << __LINE__ << std::endl;


        Eigen::MatrixXd GK = tP_ * G.transpose().eval() * (G * tP_ * G.transpose().eval() + GR).inverse();
        if (JustDebugging) std::cout << __FILE__ << ":" << __LINE__ << std::endl;

//        x_h_ = x_h_ + GK * (range_val - distance);
        x_h_.block(0,0,3,1) = x_h_.block(0,0,3,1) + GK.block(0,0,3,1)/GK.block(0,0,3,1).norm() * 0.1 * (range_val-distance) * 0.1;
        if (JustDebugging) std::cout << __FILE__ << ":" << __LINE__ << std::endl;
        std::cout << "distance :" << distance
                  << "new dist :" << (beacon_position-x_h_.block(0,0,3,1)).norm()
                  << " real :" << range_val << std::endl;
        std::cout << x_h_.transpose() << std::endl;
        std::cout << "GK:" << GK << std::endl;
        return true;
    }


};


#endif //QUICKFUSING_RANGEEKF_H
