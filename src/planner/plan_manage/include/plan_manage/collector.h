#ifndef __COLLECTOR_H
#define __COLLECTOR_H
#include <fstream>
#include <iostream>
#include <string>
#include <Eigen/Dense>


class Collector{
public:
    std::ofstream outfile;
    std::string filename;
    Collector(){}
    Collector(std::string name) : filename(name){
        outfile.open(filename);
        if(!outfile){
            outfile.close();
            std::cout << "fail to open file!" << std::endl;
            
        }else{
            std::cout << "open file" << std::endl;
            outfile << "t,pos,vel,des_pos,des_vel" << std::endl;
        }
    }
    void write(uint64_t t, Eigen::Vector3d& pos, Eigen::Vector3d& vel, Eigen::Vector3d& des_pos, Eigen::Vector3d& des_vel){
        outfile << t << ",";
		outfile << "\"[" << pos(0) << "," << pos(1) << "," << pos(2) << "]\",";
		outfile << "\"[" << vel(0) << "," << vel(1) << "," << vel(2) << "]\",";
		outfile << "\"[" << des_pos(0) << "," << des_pos(1) << "," << des_pos(2) << "]\",";
		outfile << "\"[" << des_vel(0) << "," << des_vel(1) << "," << des_vel(2) << "]\"";
		outfile << std::endl;
    }
    

private:

};


#endif