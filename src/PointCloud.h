//
// Created by Jana on 30.08.2023.
//
#include <string>
#include <cstdint>
#include <vector>
#include "Vertex.h"
#include "KdTree.h"
#include "LasDataIO.h"
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/gpu/features/normal_3d.cu>

#ifndef LASCAMPUS_POINTCLOUD_H
#define LASCAMPUS_POINTCLOUD_H


class PointCloud {
public:
    PointCloud(const std::vector<std::string>& files);

    uint32_t getVertexCount();

    pcl::PointXYZRGBNormal* getVertices();

    void kNN(const Vertex& point, size_t k,
             std::vector<KdTreeNode>* result);

//    Vertex getUTMForOpenGL(Vertex* vertex);
//
//    Vertex getWGSForOpenGL(Vertex* vertex);

private:
    const char* TAG = "PC\t";

    KdTree tree;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

    // offset is in opengl coord system!
//    float xOffset;
//    float yOffset;
//    float zOffset;


    void calculateNormals(const uint32_t& startIdx, const uint32_t& endIdx);
};


#endif //LASCAMPUS_POINTCLOUD_H
