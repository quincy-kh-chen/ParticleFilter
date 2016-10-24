//
//  main.cpp
//  Partical Filter
//
//  Created by Hsin on 10/22/16.
//  Copyright © 2016 CMU. All rights reserved.
//

#include <iostream>
#include "MyFilter.hpp"
#include "Data.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
int main(int argc, const char * argv[]) {
    // insert code here...
    std::cout << "16-831 lab2 \n";
    
    MyFilter filter;
    string log_name = "../data/log/robotdata1.log";
    string map_FilePath = "../data/map/wean.dat";
    
    // load log files and map
    filter.init(log_name, map_FilePath);
    filter.initParticles();
    filter.run();    
    
    return 0;
}
