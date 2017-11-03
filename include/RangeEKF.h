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

class RangeEKF :public MyEkf{
public:


    RangeEKF(SettingPara para):
            MyEkf(para){

    }

    bool CorrectRange(Eigen::Vector3d beacon_position,
    double range_val, double range_sigma)
    {

        return true;
    }


};


#endif //QUICKFUSING_RANGEEKF_H
