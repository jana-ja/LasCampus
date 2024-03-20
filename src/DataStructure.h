//
// Created by Jana on 30.08.2023.
//


#ifndef LASCAMPUS_DATASTRUCTURE_H
#define LASCAMPUS_DATASTRUCTURE_H

#include <vector>
#include <pcl/octree/octree_search.h>
#include "DataIO.h"
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>  // defines the PCL_INSTANTIATE_PRODUCT macro
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp> // make sure to include the .hpp file
#include <tuple>

// pcl only instantiates most common use cases, I use PointXYZRGBNormal
//PCL_INSTANTIATE_PRODUCT(NormalEstimation, ((pcl::PointXYZRGBNormal))((pcl::PointXYZRGBNormal)));


class DataStructure {
public:
    DataStructure(const std::vector<std::string> &lasFiles, const std::string &shpFile, const std::string &gmlFile, const std::string &imgFile);

    uint32_t getVertexCount();

    pcl::PointXYZRGBNormal* getVertices(){
        return cloud->data();
    }
    pcl::PointXYZ* getTangent1Vec(){
        return tangent1Vec.data();
    }
    pcl::PointXYZ* getTangent2Vec(){
        return tangent2Vec.data();
    }
    pcl::PointXY* getTexCoords(){
        return texCoords.data();
    }

    std::string getImgFile(){
        return imgFile;
    }

//    Vertex getUTMForOpenGL(Vertex* vertex);
//
//    Vertex getWGSForOpenGL(Vertex* vertex);

    float xOffset{};
    float yOffset{};
    float zOffset{};



private:

    bool colorClasses = true;
    // splats + invalid + discarded = 1
    bool colorSplats = false;
    bool colorInvalid = false; // pink und blue
    bool colorDiscarded = false; // cyan

    using Plane = pcl::PointXYZRGBNormal[3]; // three points define a plane

    const char *TAG = "PC\t";

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    int wallPointsStartIndex;

    std::vector<pcl::PointXYZ> tangent1Vec;
    std::vector<pcl::PointXYZ> tangent2Vec;


    std::vector<pcl::PointXY> texCoords;
    std::string imgFile;

    // offset is in opengl coord system!
//    float xOffset;
//    float yOffset;
//    float zOffset;


    struct Neighborhood{
        Plane plane;
        std::vector<int> pointIdc;
    };





    void adaSplats(pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree, std::vector<int>& pointClasses);

    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr kdTreePcaNormalEstimation(const uint32_t& startIdx, const uint32_t& endIdx);



    int findIndex(float border, std::vector<float> vector1);


    float adaKnnAndRadius(int k, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree, std::vector<pcl::Indices>& pointNeighbourhoods, std::vector<std::vector<float>>& pointNeighbourhoodsDistance);
    float adaNeighbourhoodsClassificationAndEpsilon(float avgRadiusNeighbourhoods, std::vector<pcl::Indices>& pointNeighbourhoods, std::vector<std::vector<float>>& pointNeighbourhoodsDistance, std::vector<int>& pointClasses);
    //    void adaUpdateNeighbourhoods(int k, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree, std::vector<pcl::Indices>& pointNeighbourhoods, std::vector<std::vector<float>>& pointNeighbourhoodsDistance, std::vector<int>& pointClasses);
    // update neighbourhoods with classification
//    float adaKnnAndAvgRadius(int k, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& treePtr, std::vector<pcl::Indices>& pointNeighbourhoods, std::vector<std::vector<float>>& pointNeighbourhoodsDistance);
    void adaNewNeighbourhoods(int k, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree, float avgRadiusNeighbourhoods, std::vector<pcl::Indices>& pointNeighbourhoods, std::vector<std::vector<float>>& pointNeighbourhoodsDistance, std::vector<int>& pointClasses);
    void adaComputeSplats(float splatGrowEpsilon, std::vector<pcl::Indices>& pointNeighbourhoods, std::vector<std::vector<float>>& pointNeighbourhoodsDistance, std::vector<int>& pointClasses);
    void adaResampling(float avgRadiusNeighbourhoods, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr& tree, std::vector<int>& pointClasses);
};


#endif //LASCAMPUS_DATASTRUCTURE_H
