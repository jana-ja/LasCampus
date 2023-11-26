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

    ShpDataIO(double maxX, double maxY, double minX, double minY);

    struct Point {
        double x, y;
    };

    /*
     * Polygon may contain one or more rings.
     * The rings are closed (the first and last vertex of a ring MUST be the same).
     * Waling along the ring in vertex order -> inside of polygon is on the right hand side.
     *  -> Polygons with only one ring always in clockwise order. Holes in Polygons are counterclockwise.
     */
    struct Polygon {
        std::vector<uint32_t> parts; // size = numParts
        std::vector<Point> points; // size = numPoints
    };

    void readShp(const std::string& path, std::vector<Polygon>* buildings);


private:

    const std::string TAG = "ShpIO\t";

    double boundsMaxX, boundsMaxY, boundsMinX, boundsMinY;


#pragma pack(push, 1) // win - tightly pack the bytes and dont start at new power of two things
    struct Header {
        // big endianess
        uint32_t fileCode;
        uint32_t unused[5];
        uint32_t fileLength; // in 16 bit words, including the header. header is 100 bytes = 50 16-bit words
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
        uint32_t contentLen; // in 16-bit words
    };

    // variable length record content for shape type 5
    struct PolygonRecContent {
        uint32_t shapeType;
        double xMin, yMin, xMax, yMax;
        uint32_t numParts; // "rings" of polygon.
        uint32_t numPoints;
        // those will be handled in vectors
//        uint32_t parts[numParts];
//        Point points[numPOints];
    };



#pragma pack(pop)

    bool isPolygonInBounds(PolygonRecContent& polygon);


};

#endif //LASCAMPUS_SHPDATAIO_H
