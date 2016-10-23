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
    std::normal_distribution<double> x_normal, y_normal, t_normal;
//    int nparticles = 100;
    for (int i = 0; i < m_NumParticles; i++){
        float delta_x = x_normal(gen);
        float delta_y = y_normal(gen);
        float delta_t = t_normal(gen);
        m_Particle[i].x = m_Particle[i].x + (m_Robot[timestamp].x - m_Robot[timestamp - 1].x) + delta_x;
        m_Particle[i].y = m_Particle[i].y + (m_Robot[timestamp].y - m_Robot[timestamp - 1].y) + delta_y;
        m_Particle[i].theta = m_Particle[i].theta + (m_Robot[timestamp].theta - m_Robot[timestamp - 1].theta) + delta_t;
    }
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
