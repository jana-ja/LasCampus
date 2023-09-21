#include <iostream>
#include "PointCloud.h"
#include "DummyData.h"
#include "Window.h"


int main() {
    std::cout << "Hello, World!" << std::endl;

    // read data
//    auto lol = PointCloud(R"(..//las//bdom50_32389_5705_1_nw_2021.las)");
    auto lol = PointCloud(R"(..//las//3dm_32_389_5705_1_nw.las)");


    // for now get dummy data
    //auto lol = DummyData();


    Window windi = Window(lol);
    //windi.setVertices(vertices);


    return 0;
}
