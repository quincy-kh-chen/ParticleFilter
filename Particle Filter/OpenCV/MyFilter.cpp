//
//  MyFilter.cpp
//  Partical_Filter
//
//  Created by Hsin on 10/22/16.
//  Copyright © 2016 CMU. All rights reserved.
//

#include "MyFilter.hpp"
#include <sstream>
#include <math.h>

#define PI 3.1415926535897932384626433832795 

void MyFilter::init(string log_FilePath, string map_FilePath) {
    // read log data (odometry and laser)
    readLog(log_FilePath);
    
    // read Map
    m_Map.read_beesoft_map(map_FilePath);
    
    // intial cv::Mat form the map read above
    cv::namedWindow(m_WindowName);
    initImage();
}

void MyFilter::initParticles() {
    for(int xx = 0; xx < m_Map.size_x; ++xx) {
        for(int yy = 0; yy < m_Map.size_y; ++yy) {
            // if the prob == 0.0; it's un occupied
            if(m_Map.prob[xx][yy] == 0.0) {
                Particle p(xx, yy, 0.0);
                m_Particle.push_back(p);
            }
        }
    }
}
void MyFilter::run() {
    while(1){
        display();
        cv::waitKey(5);
    }

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
                Robot robot_temp;
                Laser laser_temp;
                ss >> robot_temp.x >> robot_temp.y >> robot_temp.theta;
                ss >> laser_temp.x >> laser_temp.y >> laser_temp.theta;
                
                for (int i = 0; i < 180; i ++){
                    ss >> laser_temp.range[i];
                }
                m_Robot.push_back(robot_temp);
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
    std::normal_distribution<double> x_normal, y_normal, theta_normal;
    for (int i = 0; i < m_NumParticles; i++){
        float delta_x = x_normal(gen);
        float delta_y = y_normal(gen);
        float delta_theta = theta_normal(gen);
        m_Particle[i].x = m_Particle[i].x + (m_Robot[timestamp].x - m_Robot[timestamp - 1].x) / 10.0 + delta_x;
        m_Particle[i].y = m_Particle[i].y + (m_Robot[timestamp].y - m_Robot[timestamp - 1].y) / 10.0 + delta_y;
        m_Particle[i].theta = m_Particle[i].theta + (m_Robot[timestamp].theta - m_Robot[timestamp - 1].theta) / 10.0 + delta_theta;

        if (m_Particle[i].x > 800.0)  m_Particle[i].x = 800.0;
        if (m_Particle[i].y > 800.0)  m_Particle[i].y = 800.0;
        if (m_Particle[i].x < 0.0)    m_Particle[i].x = 0.0;
        if (m_Particle[i].y < 0.0)    m_Particle[i].y = 0.0;
    }
}

float MyFilter::sensorModel(float x, float mu)
{
    // Gaussian
    float sigma = 10;
    float gaussian = 1 / (sigma * sqrt(2 * PI)) * exp(-(x - mu) * (x - mu) / (2.0 * sigma * sigma));

    // exponential
    float lambda = -0.0008;
    float exponential = exp(lambda * x);

    // uniform
    float uniform = 0.25;

    return gaussian + exponential + uniform;

    // float g_max = 1.0;
    // float gaussian = g_max*exp(-((mu - x) * (mu - x)) / ((sigma_g * sigma_g) * 2.0));

    // Max range
    // float end_of_range = g_max/3; //Jane
    // float max_range = (x > 818.0) ? max_range_prob : 0.0;

    // Uniform
    // float uniform = g_max * 1.0/4.0;

    // Exponential component
    // float sigma_e = -0.0008;
    // float a = g_max*1.0/3.0;
    // float exponential = a*exp(sigma_e*x);

    // return gaussian + exponential + uniform + max_range;
}


float MyFilter::calculateWeight(Particle particle, int time) {
    float weight = 1.0;

    for (int angle = -90; angle < 90; angle++) {

        float end_point_x = particle.x + m_LaserRange * cos(particle.theta + angle * PI / 180.0);
        float end_point_y = particle.y + m_LaserRange * sin(particle.theta + angle * PI / 180.0);

        float dist_x = end_point_x - particle.x;
        float dist_y = end_point_y - particle.y;

        for (int n = 1; n <= m_NumCheck; n++) {

            float check_x = particle.x + dist_x * n / m_NumCheck;
            float check_y = particle.y + dist_y * n / m_NumCheck;

            int check_cell_x = round(check_x);
            int check_cell_y = round(check_y);

            // out of bound or hit unknown area
            if ((check_x > 800.0 || check_x < 0.0 || check_y > 800.0 || check_y < 0.0) || (m_Map.prob[check_cell_x][check_cell_y] < 0.0)) {
                
                float weight = weight * 0.1;
                break;
            }

            // hit walls
            if (m_Map.prob[check_cell_x][check_cell_y] > m_WallThres) {

                float laser_reading = m_Laser[time].range[angle + 90] / 10.0;
                float wall_dist = sqrt(pow((check_x - particle.x), 2) + pow((check_y - particle.y), 2));
                float real_wall_prob = sensorModel(laser_reading, wall_dist);
                weight *= real_wall_prob;
            }
        }

    }
    return weight;

}
// visulization
void MyFilter::initImage()
{
    m_OrigImg = cv::Mat(m_Map.size_x, m_Map.size_y, CV_32FC3);
    
    for(int xx = 0; xx < m_OrigImg.rows; ++xx) {
        for(int yy = 0; yy < m_OrigImg.cols; ++yy) {
            float pixel = 1 - m_Map.prob[xx][m_OrigImg.cols - yy];
            cv::Point3f P3f(pixel, pixel, pixel);
            m_OrigImg.at<cv::Point3f>(yy, xx) = P3f;
        }
    }
}
void MyFilter::display() {
    
    // copy original image
    m_Img = m_OrigImg.clone();
    
    // draw particles
    for(int ii = 0; ii < m_Particle.size(); ++ii) {
        cv::Point3f RED(0, 0, 255);
        int row = m_Particle[ii].y;
        int col = m_Particle[ii].x;
        m_OrigImg.at<cv::Point3f>(m_OrigImg.rows - row, col) = RED;
    }
    cout << "number of particles = " << m_Particle.size() << endl;
//    cv::cvtColor(m_Img, m_Img, cv::COLOR_GRAY2BGR);
    cv::imshow(m_WindowName ,m_Img);

//    cv::waitKey(10);
    return;
}
