//
// Created by Jana on 30.08.2023.
//
#include <string>
#include <cstdint>
#include <vector>
#include "Vertex.h"

#ifndef MASTI_POINTCLOUD_H
#define MASTI_POINTCLOUD_H



class PointCloud {
public:
    PointCloud(const std::string &path);

    uint32_t getVerticesCount();
    Vertex* getVertices();

private:
    std::vector<Vertex> vertices;

    #pragma pack(1) // win - tightly pack the bytes and dont start at new power of two things
    struct __attribute__ ((packed)) Header {  // mac
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
        uint32_t numVarLenRecords;
        uint8_t pointDataRecordFormat;
        uint16_t pointDataRecordLen;
        uint32_t numberOfPoints;
        uint32_t numPointsByReturn[5];
        double scaleX, scaleY, scaleZ;
        double offX, offY, offZ;
        double minX, minY, minZ;
        double maxX, maxY, maxZ;
    };

    // Point Data Record Format 1
#pragma pack(1)
    struct __attribute__ ((packed)) PointDRF1 {
        uint32_t x,y,z;
        uint16_t intensity;
        uint8_t  flags; // multiple bytes that are not needed and add up to eight
        uint8_t classification;
        uint8_t scanAngleRank;
        uint8_t userData;
        uint16_t pointSourceId;
        double gpsTime;
    };


    void read(const std::string &path);
};


#endif //MASTI_POINTCLOUD_H
