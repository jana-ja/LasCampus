//
// Created by Jana on 30.08.2023.
//
#include <string>
#include <cstdint>
#include <vector>
#include <pcl/octree/octree_search.h>
#include "LasDataIO.h"
#include "ShpDataIO.h"
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>  // defines the PCL_INSTANTIATE_PRODUCT macro
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp> // make sure to include the .hpp file

// pcl only instantiates most common use cases, I use PointXYZRGBNormal
//PCL_INSTANTIATE_PRODUCT(NormalEstimation, ((pcl::PointXYZRGBNormal))((pcl::PointXYZRGBNormal)));



#ifndef LASCAMPUS_POINTCLOUD_H
#define LASCAMPUS_POINTCLOUD_H


class DataStructure {
public:
    DataStructure(const std::vector<std::string> &lasFiles, const std::string &shpFiles);

    uint32_t getVertexCount();

    pcl::PointXYZRGBNormal *getVertices();

//    Vertex getUTMForOpenGL(Vertex* vertex);
//
//    Vertex getWGSForOpenGL(Vertex* vertex);

    float xOffset;
    float yOffset;
    float zOffset;

private:

    using Plane = pcl::PointXYZRGBNormal[3]; // three points define a plane

    const char *TAG = "PC\t";

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    std::vector<ShpDataIO::Polygon> buildings;

    // offset is in opengl coord system!
//    float xOffset;
//    float yOffset;
//    float zOffset;


    struct Neighborhood{
        Plane plane;
        std::vector<int> pointIdc;
    };


    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr kdTreePcaNormalEstimation(const uint32_t& startIdx, const uint32_t& endIdx);

    void robustNormalEstimation(const uint32_t &startIdx, const uint32_t &endIdx);
    static Neighborhood algo1(const float &r, const std::vector<int> &pointIdxRadiusSearch, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, int level, int (&spur)[10], int (&okay)[10]);

    void normalOrientation(const uint32_t &startIdx, const uint32_t &endIdx, pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr treePtr);

    static float isPointRightOfWall(pcl::PointXYZRGBNormal point, pcl::PointXYZRGBNormal wallPoint1, pcl::PointXYZRGBNormal wallPoint2) { // TODO inside/outside check
        float d = (wallPoint2.x - wallPoint1.x) * (point.y - wallPoint1.y) - (point.x - wallPoint1.x) * (wallPoint2.y - wallPoint1.y);
        return d;
    }

    static float signedPointPlaneDistance(const pcl::PointXYZRGBNormal& point, const Plane& plane) {
        // calc normal for plane points
        auto vec1 = vectorSubtract(plane[0], plane[1]);
        auto vec2 = vectorSubtract(plane[0], plane[2]);

        auto planeNormal = normalize(crossProduct(vec1, vec2));

        float dist = dotProduct(planeNormal, (vectorSubtract(point, plane[0])));

        return dist;
    }

    static float pointPlaneDistance(const pcl::PointXYZRGBNormal& point, const Plane& plane) {
        // calc normal for plane points
        auto vec1 = vectorSubtract(plane[0], plane[1]);
        auto vec2 = vectorSubtract(plane[0], plane[2]);

        auto planeNormal = normalize(crossProduct(vec1, vec2));

        float dist = dotProduct(planeNormal, (vectorSubtract(point, plane[0])));

        return abs(dist);
    }

    static pcl::PointXYZ normalize(pcl::PointXYZ point){
        float magnitude = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

        pcl::PointXYZ result;
        result.x = point.x / magnitude;
        result.y = point.y / magnitude;
        result.z = point.z / magnitude;
        return result;
    }

    static float dotProduct(const pcl::PointXYZ& a, const pcl::PointXYZ& b) {

        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static pcl::PointXYZ crossProduct(pcl::PointXYZ a, pcl::PointXYZ b) {
        pcl::PointXYZ result;
        result.x = (a.y * b.z) - (a.z * b.y);
        result.y = (a.z * b.x) - (a.x * b.z);
        result.z = (a.x * b.y) - (a.y * b.x);
        return result;
    }

    static pcl::PointXYZ vectorSubtract(const pcl::PointXYZRGBNormal& a, const pcl::PointXYZRGBNormal& b) {
        pcl::PointXYZ result;
        result.x = (a.x - b.x);
        result.y = (a.y - b.y);
        result.z = (a.z - b.z);
        return result;
    }

    ShpDataIO::Point getUtmForWgs(ShpDataIO::Point wgsPoint);
};


#endif //LASCAMPUS_POINTCLOUD_H
