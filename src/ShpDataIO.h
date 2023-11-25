//
// Created by Jana on 25.11.2023.
//

#ifndef LASCAMPUS_SHPDATAIO_H
#define LASCAMPUS_SHPDATAIO_H

#include <string>
#include <vector>
#include <fstream>
#include <cassert>

class ShpDataIO {

public:

    void readShp(const std::string& path);

private:

    const std::string TAG = "ShpIO\t";

#pragma pack(push, 1) // win - tightly pack the bytes and dont start at new power of two things
    struct Header {
        // big endianess
        uint32_t fileCode;
        uint32_t unused[5];
        uint32_t fileLength; // in 16 bit words, including the header
        // little endianess
        uint32_t version;
        uint32_t shapeType;
        double minX, minY, maxX, maxY; // minimum bounding rectangle of all shapes in dataset,
        double minZ, maxZ; //
        double minM, maxM; //

    };

// variable length record header
    struct VarLenRecHeader {
        uint32_t recNumber; // 1-based
        uint32_t recLen; // in 16-bit words
    };

    struct VarLenRecContent {
        uint32_t shapeType;
    };
#pragma pack(pop)

};

#endif //LASCAMPUS_SHPDATAIO_H
