//
// Created by Jana on 25.11.2023.
//

#include <iostream>
#include "ShpDataIO.h"
#include "UTM.h"

#include <climits>

template<typename T>
T swap_endian(T u) {
    static_assert(CHAR_BIT == 8, "CHAR_BIT != 8");

    union {
        T u;
        unsigned char u8[sizeof(T)];
    } source, dest;

    source.u = u;

    for (size_t k = 0; k < sizeof(T); k++)
        dest.u8[k] = source.u8[sizeof(T) - k - 1];

    return dest.u;
}


ShpDataIO::ShpDataIO(double maxX, double maxY, double minX, double minY): boundsMaxX(maxX), boundsMaxY(maxY), boundsMinX(minX), boundsMinY(minY) {}

// TODO idee: hier direkt bounding box mitgeben und nur die shapes innerhalb speichern
void ShpDataIO::readShp(const std::string& path, std::vector<Polygon>* buildings) {

    std::cout << TAG << "read shp file..." << std::endl;

    std::ifstream inf(path, std::ios::binary);

    if (inf.is_open()) {

        uint32_t bytesRead = 0;

        // read header
        Header header = Header();
        inf.read((char*) &header,
                 sizeof(header)); // cast to (char *) -> tell cpp we have some amount of bytes here/char array
        bytesRead += sizeof(header);
        // swap big endian fields
        header.fileCode = swap_endian<uint32_t>(header.fileCode);
        header.fileLength = swap_endian<uint32_t>(header.fileLength);
        auto fileLengthBytes = header.fileLength * 2; //* 16 / 8;

        std::cout << TAG << "File: " << path << std::endl;
        std::cout << TAG << "Shape Type: " << +header.shapeType << std::endl;


        // version checks
        if (header.shapeType != 5) {
            throw std::invalid_argument(
                    "Can't handle given SHP file. Only shapeType 5 (Polygon -> buildings) is allowed.");
        }


        bool readContents = true;
        while (readContents) {

            // read var length record header - TODO gleich vector erstellen wo die ganzen var len records reinkommen??
            VarLenRecHeader varLenRecHeader;
            inf.read((char*) &varLenRecHeader, sizeof(varLenRecHeader));
            bytesRead += sizeof(varLenRecHeader);
            // swap big endian fields
            varLenRecHeader.recNumber = swap_endian<uint32_t>(varLenRecHeader.recNumber);
            varLenRecHeader.contentLen = swap_endian<uint32_t>(varLenRecHeader.contentLen);
            auto contentLenBytes = varLenRecHeader.contentLen * 2; //* 16 / 8;
//            std::cout << TAG << "var len rec len: " << +contentLenBytes << std::endl;


            // read var length record content
            PolygonRecContent polygonRecContent;

            // read shape type
            // all shapes in file have the same shape type (5 here) or null (0)
            inf.read((char*) &polygonRecContent, sizeof(PolygonRecContent::shapeType));
            bytesRead += sizeof(PolygonRecContent::shapeType);
//            std::cout << TAG << "var len rec shape type: " << +polygonRecContent.shapeType << std::endl;
            // skip if null shape
            if (polygonRecContent.shapeType == 0) {
                std::cout << TAG << "Skipping Null Shape" << std::endl;
                continue;
            }
            // read polygon content
            inf.read((char*) &polygonRecContent + sizeof(PolygonRecContent::shapeType),
                     sizeof(polygonRecContent) - sizeof(PolygonRecContent::shapeType));
            bytesRead += sizeof(polygonRecContent) - sizeof(PolygonRecContent::shapeType);


            // check if polygon lies within bounds of las data
            if(isPolygonInBounds(polygonRecContent)) {

                // read part indices and points of all parts
                Polygon polygon;
                // part indices
                uint32_t partIndex;
                for (auto i = 0; i < polygonRecContent.numParts; i++) {
                    inf.read((char*) &partIndex, sizeof(uint32_t));
                    bytesRead += sizeof(uint32_t);
                    polygon.parts.push_back(partIndex); // push back makes copy
                }
                // points of all parts
                Point point;
                for (auto i = 0; i < polygonRecContent.numPoints; i++) {
                    inf.read((char*) &point, sizeof(Point));
                    bytesRead += sizeof(Point);
                    // convert point from w84 to utm zone 32n TODO is precise enough?
                    double utmX, utmZ;
                    LatLonToUTMXY(point.z, point.x, 32, utmX, utmZ);
                    point.x = utmX;
                    point.z = utmZ;
                    polygon.points.push_back(point);
                }

                buildings->push_back(polygon);

            } else {

                // skip this polygon
                auto remainingBytesPolygon = contentLenBytes - sizeof(PolygonRecContent);
                inf.seekg(remainingBytesPolygon, std::ios_base::cur); // skip to point tree
                bytesRead += remainingBytesPolygon;

            }

            // test if file is finished
//            std::cout << TAG << "bytesRead: " << +bytesRead << std::endl;
            if(bytesRead >= fileLengthBytes){
                readContents = false;
            }

        }

        if (!inf.good())
            throw std::runtime_error("Reading .shp ran into error");

        std::cout << TAG << "finished reading shp file" << std::endl;


    } else {
        throw std::runtime_error("Can't find .shp file");
    }
}

bool ShpDataIO::isPolygonInBounds(ShpDataIO::PolygonRecContent& polygon) {
    if (polygon.xMin > boundsMaxX || polygon.xMax < boundsMinX) {
        return false; // out of bounds in x direction
    }
    if (polygon.yMin > boundsMaxY || polygon.yMax < boundsMinY) {
        return false; // out of bounds in y direction
    }
    return true;
}

