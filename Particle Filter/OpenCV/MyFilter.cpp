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
    m_Map.read_beesoft_map(map_FilePath);
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
                m_Odom.push_back(odom_temp);
            }
            // laser
            else if (line[0] == 'L'){
                Laser laser_temp;
                ss >> laser_temp.x >> laser_temp.y >> laser_temp.theta;
                ss >> laser_temp.x_L >> laser_temp.y_L >> laser_temp.theta_L;
                
                for (int i = 0; i < 180; i ++){
                    ss >> laser_temp.range[i];
                }
                
                m_Laser.push_back(laser_temp);
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
void MyFilter::updateMotion(int timestamp){
    std::default_random_engine gen;
    std::normal_distribution<double> x_normal, y_normal, t_normal;
    int nparticles = 100;
    for (int i = 0; i < nparticles; i++){
        float delta_x = x_normal(gen);
        float delta_y = y_normal(gen);
        float delta_t = t_normal(gen);
        float x = m_Particle[i].x + (m_Laser[timestamp].x - m_Laser[timestamp - 1].x) + delta_x;
        float y = m_Particle[i].y + (m_Laser[timestamp].y - m_Laser[timestamp - 1].y) + delta_y;
        float theta = m_Particle[i].theta + (m_Laser[timestamp].theta - m_Laser[timestamp - 1].theta) + delta_t;
        m_Particle[i] = Particle(x, y, theta);
    }
}
