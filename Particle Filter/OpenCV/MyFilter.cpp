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
    m_Map.read_beesoft_map(map_FilePath, m_Particle);
    
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
    // while(1){
    for (int i = 1; i < m_Robot.size(); i++) {
        cout << i << endl;
        updateMotion(i);
        calculateWeight(m_Particle, i);
        resampleParticles(m_Particle);
//        resample_LowVar();
        // visualize();
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
            string c;
            ss >> c;
//            cout << value << endl;
            // odom
            if (line[0] == 'O') {
                Odometry odom_temp;
                ss >> odom_temp.x >> odom_temp.y >> odom_temp.theta >> odom_temp.ts;
                m_Odom.push_back(odom_temp);
                
//                cout << "robot x = " << m_Robot.back().x << endl;
            }
            // laser
            else if (line[0] == 'L'){
                Robot robot_temp;
                Laser laser_temp;
//                ss >> robot_temp.x >> robot_temp.y >> robot_temp.theta;
//                ss >> laser_temp.x >> laser_temp.y >> laser_temp.theta;
                ss >> c; robot_temp.x = stof(c) ;
                ss >> c; robot_temp.y = stof(c);
                ss >> c; robot_temp.theta = stof(c);
                
                ss >> c; laser_temp.x = stof(c);
                ss >> c; laser_temp.y = stof(c);
                ss >> c; laser_temp.theta = stof(c);
                for (int i = 0; i < 180; i ++){
                    ss >> c;
                    laser_temp.range[i] = stof(c);
                }
                m_Robot.push_back(robot_temp);
                m_Laser.push_back(laser_temp);
                
                cout << "robot :" << m_Robot.back().x << ", "<< m_Robot.back().y << ", "<< m_Robot.back().theta << endl;

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
//        cout << "x = " << m_Particle[i].x << ", y = " << m_Particle[i].y << endl;
    }
}

float MyFilter::sensorModel(float x, float mu)
{
    // Gaussian
    // float sigma = 10;
    // float gaussian = 1 / (sigma * sqrt(2 * PI)) * exp(-(x - mu) * (x - mu) / (2.0 * sigma * sigma));

    // // exponential
    // float lambda = -0.0008;
    // float exponential = exp(lambda * x);

    // // uniform
    // float uniform = 0.25;

    // return gaussian + exponential + uniform;
    // Gaussian component
    float sigma_g = 10;

    float g_max = 1.0;
    float gaussian = g_max*exp(-((mu - x) * (mu - x)) / ((sigma_g * sigma_g) * 2.0));

    // Max range component
    float max_range_prob = g_max/3;
    float max_range = (x > 818.0) ? max_range_prob : 0.0;

    // Uniform component
    float uniform = g_max * 1.0/4.0;

    // Exponential component
    float sigma_e = -0.0008;
    float a = g_max*1.0/3.0;
    float exponential = a*exp(sigma_e*x);

    return gaussian + exponential + uniform + max_range;
}
const pair<float, float>& transform(const float& x, const float& y)  {
    pair<float, float> p;
    p.first = (800 - y);
    p.second = x;
    return p;
}

void MyFilter::calculateWeight(vector<Particle>& particles, int time) {

    for (int i = 0; i < m_NumParticles; i++){
//        cout << "particles[" << i << "].weight = " << particles[i].weight << endl;
        // pair<float, float> p = transform(particles[i].x, particles[i].y);
        // float end_point_orig_x = p.first;
        // float end_point_orig_y = p.second;
        pair<float, float> ps = transform(particles[i].x, particles[i].y);
        float start_point_x = ps.first;
        float start_point_y = ps.second;
//        cout << "particles[" << i << "].weight = " << particles[i].weight << endl;
        for (int angle = -90; angle < 90; angle+= 5) {
        
            float x_Laser = particles[i].x + m_LaserRange * cos(particles[i].theta + angle * PI / 180.0) / 10.0;
            float y_Laser = particles[i].y + m_LaserRange * sin(particles[i].theta + angle * PI / 180.0) / 10.0;

            pair<float, float> p = transform(x_Laser, y_Laser);
            float end_point_x = p.first;
            float end_point_y = p.second;

           // float end_point_x = particles[i].x + m_LaserRange * cos(particles[i].theta + angle * PI / 180.0);
           // float end_point_y = particles[i].y + m_LaserRange * sin(particles[i].theta + angle * PI / 180.0);

            // float end_point_x = end_point_orig_x + float(m_LaserRange) * cos(particles[i].theta + angle * PI / 180.0);
            // float end_point_y = end_point_orig_y + float(m_LaserRange) * sin(particles[i].theta + angle * PI / 180.0);
            
            // float dist_x = end_point_x - particles[i].x;
            // float dist_y = end_point_y - particles[i].y;
            float dist_x = end_point_x - start_point_x;
            float dist_y = end_point_y - start_point_y;
            if(angle == -70) {
//                cout << "particles[" << i << "].weight = " << particles[i].weight << endl;
            }
            for (int n = 1; n <= m_NumCheck; n++) {

                float check_x = start_point_x + dist_x * float(n) / m_NumCheck;
                float check_y = start_point_y + dist_y * float(n) / m_NumCheck;

                int check_cell_x = round(check_x);
                int check_cell_y = round(check_y);

                // out of bound or hit unknown area
                if ((check_cell_x >= 800 || check_cell_x < 0 || check_cell_y >= 800 || check_cell_y < 0) || (m_Map.prob[check_cell_x][check_cell_y] < 0.35)) {
//                    cout << "particles[" << i << "].weight = " << particles[i].weight << endl;
                   particles[i].weight += log(0.8);
                    break;
                } else {
//                    cout << "in range!" << endl;
                }
//                cout << "particles[" << i << "].weight = " << particles[i].weight << endl;
                // hit walls
                if (m_Map.prob[check_cell_x][check_cell_y] < m_WallThres) {

                    float laser_reading = m_Laser[time].range[angle + 90] / 10.0;
//                    float wall_dist = sqrt(pow((check_x - particles[i].x), 2) + pow((check_y - particles[i].y), 2));
                    float wall_dist = sqrt(pow((check_x - start_point_x), 2) + pow((check_y - start_point_y), 2));
//                    float real_wall_prob = pow(sensorModel(laser_reading, wall_dist), 0.1);
                    float real_wall_prob = sensorModel(laser_reading, wall_dist)* 10;
//                    cout << "real_wall_prob = " << real_wall_prob << endl;
                    // particles[i].weight = particles[i].weight * 1.1 * real_wall_prob;
//                    m_Particle[i].weight += log(real_wall_prob);

                }
            }
        }
        m_Particle[i].weight = exp(m_Particle[i].weight);
//        cout << "particles[" << i << "].weight = " << particles[i].weight << endl;
    }

}

void MyFilter::resampleParticles(vector<Particle>& particles)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
  // Set up distribution
    std::vector<int> intervals;
    std::vector<float> weights;
    
    for (int p = 0; p <= particles.size(); ++p) {
        weights.push_back(particles[p].weight);
        intervals.push_back(p);
    }

    std::piecewise_constant_distribution<> dist(begin(intervals),
                                                end(intervals),
                                                begin(weights));

  // Copy the current particles and clear the vector
    vector<Particle> oldParticles = particles;
    particles.clear();

    for (int i = 0; i < m_NumParticles; ++i)
    {
        // Generate random number using gen, distributed according to dist
        int r = (int) dist(generator);
        if(r < 0 || r >= oldParticles.size()) {
//            cout << r << endl;
        } else {
            // Push the new particle into the vector
            particles.push_back(oldParticles[r]);
        }
    }

    return;
}
void MyFilter::resample_LowVar() {
    float temp = (float)rand() / (float)RAND_MAX;
    float r =  temp * ((float)1/m_Particle.size());
    float curWeight =  m_Particle[0].weight;
    int idx = 0;
    
    vector<Particle> copy = m_Particle;
    m_Particle.clear();
//    for(int ii = 0; ii < copy.size(); ++ii) {
    for(int ii = 0; ii < m_NumParticles; ++ii) {
        //        Particle newParticle;
        float newWeight = r + float(ii)/copy.size();
        while(curWeight < newWeight) {
            ++idx;
            if(idx >= copy.size()) {
                cout << idx << " ERROR" << endl;
            }
            curWeight += copy[idx].weight;
        }
        //        newParticle = m_Particle[idx];
        cout << idx << endl;
//        cout << copy.size() << endl;
        m_Particle.push_back(copy[idx]);
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
//    for(int ii = 0; ii < m_Particle.size(); ++ii) {
    for(int ii = 0; ii < m_NumParticles; ++ii) {
        cv::Point3f RED(0, 0, 255);
        
//        int row = m_OrigImg.rows - m_Particle[ii].y;
//        int col = m_Particle[ii].x;
        pair<float, float> p = transform(m_Particle[ii].x, m_Particle[ii].y);
        int row = p.first;
        int col = p.second;
//        cout << row << " " << col << endl;
        if(col >= 0 && col < 800 & row >= 0 && row < 800) {
            m_Img.at<cv::Point3f>(row, col) = RED;
        } else {
            cout << "row = " << row << ", col = " << col << endl;
        }
    }
//    cout << "number of particles = " << m_Particle.size() << endl;
//    cv::cvtColor(m_Img, m_Img, cv::COLOR_GRAY2BGR);
    cv::imshow(m_WindowName ,m_Img);

//    cv::waitKey(10);
    return;
}
