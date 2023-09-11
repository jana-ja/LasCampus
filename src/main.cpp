#include <iostream>
#include "PointCloud.h"
#include "DummyData.h"
#include "loadShader.h"
#include "Window.h"


int main() {
    std::cout << "Hello, World!" << std::endl;

    // read data
    auto lol = PointCloud(R"(..//las//3dm_32_389_5705_1_nw.las)");


    // for now get dummy data
    //auto lol = DummyData();


    Window windi = Window(lol.getVertices(), lol.getVerticesCount());
    //windi.setVertices(vertices);


    return 0;
}
