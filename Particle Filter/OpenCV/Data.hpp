//
//  Data.hpp
//  Partical Filter
//
//  Created by Hsin on 10/22/16.
//  Copyright © 2016 CMU. All rights reserved.
//

#ifndef Data_hpp
#define Data_hpp

#include <stdio.h>
#include <stdlib.h>
#include <string>

//#include <vector>

class Laser {
public:
    float x;
    float y;
    float theta;
    
    // coordinates of the *laser* in standard odometry frame when the laser reading was taken (interpolated)
    float x_L;
    float y_L;
    float theta_L;
    float range[180]; // range readings from laser
    
    // timestamp of laser reading
    float ts;
};

class Odometry {
public:
    float x;
    float y;
    float theta;
    float ts;
};

class Map {
public:
    int min_x, max_x;
    int min_y, max_y;
    int size_x, size_y;
    float offset_x, offset_y;
    int resolution;
    float prob[800][800]; // need modify
    float **cells;
    
    // function
    int read_beesoft_map(std::string log_FilePath);
    void new_hornetsoft_map(Map *map, int size_x, int size_y);
};

class Particle {
public:
    float x, y, theta;
    Particle();
    Particle(float origX, float origY, float origTheta);
};
#endif /* Data_hpp */
