//
//  MyFilter.hpp
//  Partical_Filter
//
//  Created by Hsin on 10/22/16.
//  Copyright © 2016 CMU. All rights reserved.
//

#ifndef MyFilter_hpp
#define MyFilter_hpp

#include <stdio.h>
#include "Data.hpp"
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <random>
#endif /* MyFilter_hpp */

using namespace std;

class MyFilter {
public:
    
    // map class
    Map m_Map;
    // laser data
    vector<Laser> m_Laser;
    // Odometry data
    vector<Odometry> m_Odom;
    // particles
    vector<Particle> m_Particle;
    
    void init(string log_FilePath, string map_FilePath);
    void readLog(string log_FilePath); // read log data
    void updateMotion(int timestamp); // motion model
};
