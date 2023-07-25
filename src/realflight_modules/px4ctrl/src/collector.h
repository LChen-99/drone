#ifndef __COLLECTOR_H
#define __COLLECTOR_H
#include <fstream>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "PX4CtrlFSM.h"

class Collector{
public:
    std::ofstream outfile;
    std::string filename;
    Collector(){}
    Collector(std::string name) : filename(name){
        outfile.open(filename);
        if(!outfile){
            outfile.close();
            ROS_INFO("fail to open file!");
            
        }else{
            ROS_INFO("open file");
            outfile << "t,p,v,p_d,v_d,q,pwm,a,f_obs,f_mea,T_sp,q_sp,thr2acc,volt,bat_percentage" << std::endl;
        }
    }
    

private:

};


#endif