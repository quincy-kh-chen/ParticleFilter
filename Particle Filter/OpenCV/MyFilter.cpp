//
//  MyFilter.cpp
//  Partical_Filter
//
//  Created by Hsin on 10/22/16.
//  Copyright © 2016 CMU. All rights reserved.
//

#include "MyFilter.hpp"
#include <sstream>

void MyFilter::init(string log_FilePath, string map_FilePath) {
    // read log data (odometry and laser)
    readLog(log_FilePath);
    
    // read Map
    map.read_beesoft_map(map_FilePath);
}
void MyFilter::readLog(string log_FilePath) {
    ifstream logFile(log_FilePath);
    
    if (logFile.is_open()) {
        string line;
        while ( getline (logFile,line) ) {

            istringstream ss(line);
            float value;
            ss >> value;
            // odom
            if (line[0] == 'O') {
                Odometry odom_temp;
                ss >> odom_temp.x >> odom_temp.y >> odom_temp.theta >> odom_temp.ts;
                odom.push_back(odom_temp);
            }
            // laser
            else if (line[0] == 'L'){
                Laser laser_temp;
                ss >> laser_temp.x >> laser_temp.y >> laser_temp.theta;
                ss >> laser_temp.x_L >> laser_temp.y_L >> laser_temp.theta_L;
                
                for (int i = 0; i < 180; i ++){
                    ss >> laser_temp.range[i];
                }
                
                laser.push_back(laser_temp);
            }
            else {
                cout << "Unexpected data, not odometry nor laser : " << log_FilePath << "\n";
            }
        }
        logFile.close();
    } else {
        cout << "Can't open file :" << log_FilePath << "\n";
    }
    return;
}
