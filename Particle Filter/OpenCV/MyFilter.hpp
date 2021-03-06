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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#endif /* MyFilter_hpp */

using namespace std;

class MyFilter {
public:
    // parameters
    int m_NumParticles = 10000; // number fo particles
    float m_WallThres = 0.9; // the threshold for considering as a wall
    float m_LaserRange = 400; // max laser range
    float m_NumCheck = 5; // number of points along a laser line
    float m_Resolution = 10.0; // convert from log coordinate to map/image coordinate

    // openCV
    cv::Mat m_OrigImg; // original image
    cv::Mat m_Img;
    string m_WindowName = "Map"; // windowName for display image
    
    // map class
    Map m_Map;
    // robot position from laser data
    vector<Robot> m_Robot;
    // laser data
    vector<Laser> m_Laser;
    // Odometry data
    vector<Odometry> m_Odom;
    // particles
    vector<Particle> m_Particle;
    
    void init(string log_FilePath, string map_FilePath);
    void initParticles();
    void initImage();
    void display();
    void readLog(string log_FilePath); // read log data
    void updateMotion(int timestamp); // motion model
    float sensorModel(float x, float mu);
    void run();
    void calculateWeight(vector<Particle>& particles, int time);
    void resampleParticles(vector<Particle>& particles);
    void resample_LowVar();
};
