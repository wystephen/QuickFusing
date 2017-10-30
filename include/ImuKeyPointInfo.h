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
// Created by steve on 17-10-30.
//

#ifndef QUICKFUSING_IMUKEYPOINTINFO_H
#define QUICKFUSING_IMUKEYPOINTINFO_H


#include <Eigen/Dense>
#include <Eigen/Geometry>

class ImuKeyPointInfo {
public:
    ImuKeyPointInfo(const int &index, const Eigen::MatrixXd &data) {
        index_ = index;
        time_ = data(0,0);
        data_vec_ = data;

    }


    int index_;
    double time_ = 0.0;
    Eigen::MatrixXd data_vec_ = Eigen::Matrix<double, 11, 1>::Zero();


};


#endif //QUICKFUSING_IMUKEYPOINTINFO_H
