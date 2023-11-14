//
// Created by Jana on 30.08.2023.
//

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <assert.h>
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


//    tree = KdTree(vertices);
//    buildTree(vertices);

//    calculateNormals();
}

uint32_t PointCloud::getVertexCount() {
    return (uint32_t) cloud->width;
}

pcl::PointXYZRGBNormal *PointCloud::getVertices() {
    return cloud->data();// vertices.data();
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

void PointCloud::kNN(const Vertex &point, size_t k, std::vector<KdTreeNode> *result) {
    tree.kNN(point, k, result);
}

void PointCloud::calculateNormals(const uint32_t& startIdx, const uint32_t& endIdx) { // TODO use indices
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << TAG << "start normal calculation" << std::endl;

    auto blub = cloud->points[0];
    std::cout << sizeof(blub) << std::endl;

//
////    // Placeholder for the 3x3 covariance matrix at each surface patch
////    Eigen::Matrix3f covariance_matrix;
////    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
////    Eigen::Vector4f xyz_centroid;
////    // Estimate the XYZ centroid
////    pcl::compute3DCentroid(cloud, xyz_centroid);
////    // Compute the 3x3 covariance matrix
////    pcl::computeCovarianceMatrix(cloud, xyz_centroid, covariance_matrix);
////    // TODO solve normal orientation
//
////
////    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
////    cloudPtr.pu;
//
//
//    // Create the normal estimation class, and pass the input dataset to it
////    pcl::gpu::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//    pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
////    pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> ne2;
//    //ne.setNumberOfThreads(8);
//    ne.setInputCloud(cloud);
//
//    // Create an empty kdtree representation, and pass it to the normal estimation object.
//    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
//    ne.setSearchMethod(tree);
//
//
//    // Use all neighbors in a sphere of radius 300cm
////    ne.setRadiusSearch(3);
//    pcl::PointXYZRGBNormal bla;
//
//    bla = cloud->points[0];
//    ne.setKSearch(6);
//
//    // Compute the features
////    ne.compute(*normals);
//    ne.compute(*cloud);
//
//
//    // normals->size () should have the same size as the input cloud->size ()*
//
//
//    bla = cloud->points[0];
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << TAG << "finished normal calculation in " << duration.count() << "s" << std::endl;
}