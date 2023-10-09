#include <iostream>
#include "PointCloud.h"
#include "getDummyData.h"
#include "loadShader.h"
#include "Window.h"

// Include GLEW. Always include it before gl.h and glfw3.h, since it's a bit magic.
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <cstdlib>
#include <iostream>

int main() {
    std::cout << "Hello, World!" << std::endl;

    // read data
    //PointCloud(R"(..\las\3dm_32_389_5705_1_nw.las)");

    // for now get dummy data
    Vertex* vertices = getDummyVertices();

    Window windi = Window();
    windi.setVertices(vertices);


    return 0;
}
