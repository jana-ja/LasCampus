//
// Created by Jana on 30.08.2023.
//

#include <iostream>
#include <stdexcept>
#include <Eigen/Core>
#include <chrono>
#include "util.h"
#include "PointCloud.h"


using namespace std;

PointCloud::PointCloud(const std::vector<std::string> &files) {

    LasDataIO io = LasDataIO();


    std::string dir = ".." + PATH_SEPARATOR + "las" + PATH_SEPARATOR;
    // in cache muss: pointrecformat, anzahl points, offset

    uint32_t startIdx = 0;
    uint32_t endIdx;
    uint32_t pointCount;
    std::cout << TAG << "begin loading data" << std::endl;
    for (const auto &file: files) {

        // get points
        io.readLas(dir + file, cloud, &pointCount);

        endIdx = pointCount;

        // get normals
        std::string normalFile = file;
        normalFile.replace(normalFile.end()-3, normalFile.end()-1, "normal");
        if(!io.readNormalsFromCache(dir + normalFile, cloud, startIdx, endIdx)){
//            calculateNormals(startIdx, endIdx); // TODO use idx
//            io.writeNormalsToCache(dir + normalFile, cloud, startIdx, endIdx); // TODO temporarily not used
        }

        startIdx += pointCount;
    }
    std::cout << TAG << "loading data successful" << std::endl;
}


void PointCloud::calculateNormals(const uint32_t& startIdx, const uint32_t& endIdx) { // TODO use indices
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << TAG << "start normal calculation" << std::endl;





    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished normal calculation in " << duration.count() << "s" << std::endl;
}


//Vertex PointCloud::getUTMForOpenGL(Vertex *vertexOpenGL) {
//    // TODO offset is float, losing precision
//    return Vertex{vertexOpenGL->x + xOffset, vertexOpenGL->y + yOffset, vertexOpenGL->z + zOffset};
//}
//
//Vertex PointCloud::getWGSForOpenGL(Vertex *vertex) {
//    // TODO offset is float, losing precision
//
//    // wert in utm holen, dann:
//
//    // zone number: 60 zones, each 6 degrees of longitude (horizontal stripes), number is consistent in horizontal stripes
//    // zone letter: 20 zones, each 8 degrees of latitude (vertical stripes), letter is consistent in vertical stripes
//    // x wert zwischen 100.000 und 899.999 meter in zone
//    // y wert ist entfernugn vom äquator (zumindest auf nordhalbkugel)
//    return Vertex();
//}

uint32_t PointCloud::getVertexCount() {
    return (uint32_t) cloud->width;
}

pcl::PointXYZRGBNormal *PointCloud::getVertices() {
    return cloud->data();// vertices.data();
}