//
// Created by Jana on 30.08.2023.
//
#include <string>
#include <cstdint>
#include <vector>
#include <pcl/octree/octree_search.h>
#include "LasDataIO.h"
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#ifndef LASCAMPUS_POINTCLOUD_H
#define LASCAMPUS_POINTCLOUD_H


class DataStructure {
public:
    DataStructure(const std::vector<std::string> &files);

    uint32_t getVertexCount();

    pcl::PointXYZRGBNormal *getVertices();

//    Vertex getUTMForOpenGL(Vertex* vertex);
//
//    Vertex getWGSForOpenGL(Vertex* vertex);
    static std::vector<int> algo1(const float &r, const std::vector<int> &pointIdxRadiusSearch, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

private:
    using Neighborhood = std::vector<int>; // vector of point indices
    using Plane = pcl::PointXYZRGBNormal[3]; // three points define a plane

    const char *TAG = "PC\t";

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

    // offset is in opengl coord system!
//    float xOffset;
//    float yOffset;
//    float zOffset;

    static float pointPlaneDistance(pcl::PointXYZRGBNormal point, Plane plane) {
        // calc normal for plane points
        auto vec1 = vectorSubtract(plane[0], plane[1]);
        auto vec2 = vectorSubtract(plane[0], plane[2]);

        auto planeNormal = normalize(crossProduct(vec1, vec2));

        float dist = dotProduct(planeNormal, (vectorSubtract(point, plane[0])));

        return dist;
    }

    static pcl::PointXYZ normalize(pcl::PointXYZ point){
        float magnitude = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

        pcl::PointXYZ result;
        result.x = point.x / magnitude;
        result.y = point.y / magnitude;
        result.z = point.z / magnitude;
        return result;
    }

    static float dotProduct(pcl::PointXYZ a, pcl::PointXYZ b) {

        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static pcl::PointXYZ crossProduct(pcl::PointXYZ a, pcl::PointXYZ b) {
        pcl::PointXYZ result;
        result.x = (a.y * b.z) - (a.z * b.y);
        result.y = (a.z * b.x) - (a.x * b.z);
        result.z = (a.x * b.y) - (a.y * b.x);
        return result;
    }

    static pcl::PointXYZ vectorSubtract(pcl::PointXYZRGBNormal a, pcl::PointXYZRGBNormal b) {
        pcl::PointXYZ result;
        result.x = (a.x - b.x);
        result.y = (a.y - b.y);
        result.z = (a.z - b.z);
        return result;
    }


    void calculateNormals(const uint32_t &startIdx, const uint32_t &endIdx);

};


#endif //LASCAMPUS_POINTCLOUD_H
