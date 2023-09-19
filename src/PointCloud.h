//
// Created by Jana on 30.08.2023.
//
#include <string>
#include <cstdint>
#include <vector>
#include "Vertex.h"

#ifndef LASCAMPUS_POINTCLOUD_H
#define LASCAMPUS_POINTCLOUD_H


class PointCloud {
public:
    PointCloud(const std::string &path);

    uint32_t getVerticesCount();

    Vertex *getVertices();

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
        double maxX, minX;
        double maxY, minY;
        double maxZ, minZ;
    };

    // variable length record header
#pragma pack(1)
    struct __attribute__ ((packed)) VarLenRecHeader {
        uint16_t reserved; // unsigned short - 2 bytes
        char userid[16]; // user which created the var len rec. // can be LASF_Spec or LASF_Projection in this case
        uint16_t recordId; // depends on userid. 34735 for userId LASF_Projection is GeoKeyDirectoryTag and is mandatory here.
        uint16_t recordLenAfterHeader;
        char description[32];
    };

#pragma pack(1)
    struct __attribute__ ((packed)) GeoKeyDirectoryTag {
        uint16_t wKeyDirectoryVersion; // always 1
        uint16_t wKeyRevision; // always 1
        uint16_t wMinorRevision; // always 0
        uint16_t wNumberOfKeys;
    };

#pragma pack(1)
    struct __attribute__ ((packed)) GeoKeyEntry {
        uint16_t wKeyId; // id from GeoTiff specification
        uint16_t wTiffTagLocation; // 0 -> data is in wValueOffset.
                                    // 34736 -> wValueOffset is index of data in GeoDoubleParamsTag record.
                                    // 34767 -> wValueOffset is index of data in GeoAsciiParamsTag record.
        uint16_t wCount; // only relevant for GeoAsciiParamsTag, 1 otherwise
        uint16_t wValueOffset; // content depends on wTiffTagLocation
    };

    // Point Data Record Format 1
#pragma pack(1)
    struct __attribute__ ((packed)) PointDRF1 {
        uint32_t x, y, z;
        uint16_t intensity;
        uint8_t flags; // multiple bytes that are not needed and add up to eight
        uint8_t classification;
        uint8_t scanAngleRank;
        uint8_t userData;
        uint16_t pointSourceId;
        double gpsTime;
    };


    void read(const std::string &path);
};


#endif //LASCAMPUS_POINTCLOUD_H
