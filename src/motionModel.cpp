#include <iostream>
#include <random>
using namespace std;

void updateMotion(int timestamp){
    std::default_random_engine gen;
    std::normal_distribution<double> x_normal, y_normal, theta_normal;
    int nparticles = 100;
    for (int i = 0; i < nparticles; i++){
        float delta_x = x_normal(gen);
        float delta_y = y_normal(gen);
        float delta_theta = theta_normal(gen);
        float x = particle[i].x + (laser[timestamp].x - laser[timestamp - 1].x) + delta_x;
        float y = particle[i].y + (laser[timestamp].y - laser[timestamp - 1].y) + delta_y;
        float theta = particle[i].theta + (laser[timestamp].theta - laser[timestamp - 1].theta) + delta_theta;

        particles[i] = Particle(x, y, theta);
    }
    return;
}

int main(){
    std::default_random_engine generator;
    std::normal_distribution<double> xy_normal;
    return 0;
}