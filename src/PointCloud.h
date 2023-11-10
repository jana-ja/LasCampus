//
// Created by Jana on 30.08.2023.
//
#include <string>
#include <cstdint>
#include <vector>
#include "Vertex.h"
#include "KdTree.h"
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/gpu/features/normal_3d.cu>

#ifndef LASCAMPUS_POINTCLOUD_H
#define LASCAMPUS_POINTCLOUD_H


class PointCloud {
public:
    PointCloud(const std::vector<std::string>& files);

    uint32_t getVertexCount();

    pcl::PointNormal* getVertices();
    pcl::Normal* getNormals();

    void kNN(const Vertex& point, size_t k,
             std::vector<KdTreeNode>* result);

//    Vertex* getColorVertices();

    Vertex getUTMForOpenGL(Vertex* vertex);

    Vertex getWGSForOpenGL(Vertex* vertex);

//    bool hasColor();

private:
    const char* TAG = "PC\t";

    KdTree tree;
//    std::vector<Vertex> vertices;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    //std::vector<ColorVertex> colorVertices;

    // offset is in opengl coord system!
    float xOffset;
    float yOffset;
    float zOffset;

    int pointRecFormat;

    bool firstFile = true;

#pragma pack(push,1) // win - tightly pack the bytes and dont start at new power of two things
    struct Header {
        char magic[4];
        uint16_t fileSourceId; // unsigned short - 2 bytes
        uint16_t globalEncoding;
        uint32_t guidData1; // unsigned long - 4 bytes
        uint16_t guidData2;
        uint16_t guidData3;
        uint8_t guidData4[8]; // chars that are numbers
        uint8_t versionMaj, versionMin;
        char systemIdentifier[32];
        char genSoftware[32];
        uint16_t creationDay, creationYear;
        uint16_t headerSize;
        uint32_t pointDataOffset;
        uint32_t numVarLenRecords = 2;
        uint8_t pointDataRecordFormat;
        uint16_t pointDataRecordLen;
        uint32_t numberOfPoints;
        uint32_t numPointsByReturn[5];
        double scaleX, scaleY, scaleZ;
        double offX, offY, offZ;
        double maxX, minX;
        double maxY, minY;
        double maxZ, minZ;
    };

    // variable length record header
//#pragma pack(1)
    struct VarLenRecHeader {
        uint16_t reserved; // unsigned short - 2 bytes
        char userid[16]; // user which created the var len rec. // can be LASF_Spec or LASF_Projection in this case
        uint16_t recordId; // depends on userid. 34735 for userId LASF_Projection is GeoKeyDirectoryTag and is mandatory here.
        uint16_t recordLenAfterHeader;
        char description[32];
    };

//#pragma pack(1)
    struct GeoKeyEntry {
        uint16_t wKeyId; // id from GeoTiff specification
        uint16_t wTiffTagLocation; // 0 -> tree is in wValueOffset.
                                    // 34736 -> wValueOffset is index of tree in GeoDoubleParamsTag record.
                                    // 34767 -> wValueOffset is index of tree in GeoAsciiParamsTag record.
        uint16_t wCount; // only relevant for GeoAsciiParamsTag, 1 otherwise
        uint16_t wValueOffset; // content depends on wTiffTagLocation
    };
//
//#pragma pack(1)
    struct GeoKeyDirectoryTag {
        uint16_t wKeyDirectoryVersion; // always 1
        uint16_t wKeyRevision; // always 1
        uint16_t wMinorRevision; // always 0
        uint16_t wNumberOfKeys;
        std::vector<GeoKeyEntry> entries;
    };


    // Point Data Record Format 1
//#pragma pack(1)
    struct PointDRF1 {
        uint32_t x, y, z;
        uint16_t intensity;
        uint8_t flags; // multiple bits that are not needed and add up to eight
        uint8_t classification;
        uint8_t scanAngleRank;
        uint8_t userData;
        uint16_t pointSourceId;
        double gpsTime;
    };

    // Point Data Record Format 2
//#pragma pack(1)
//    struct PointDRF2 {
//        uint32_t x, y, z;
//        uint16_t intensity;
//        uint8_t flags; // multiple bits that are not needed and add up to eight
//        uint8_t classification;
//        uint8_t scanAngleRank;
//        uint8_t userData;
//        uint16_t pointSourceId;
//        uint16_t red;
//        uint16_t green;
//        uint16_t blue;
//    };


    void read(const std::string &path);

    void calculateNormals();

    void buildTree(std::vector<Vertex> vertices);
};

#pragma pack(pop)

#endif //LASCAMPUS_POINTCLOUD_H
