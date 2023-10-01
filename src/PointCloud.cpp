//
// Created by Jana on 30.08.2023.
//

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <assert.h>
#include "PointCloud.h"

using namespace std;

PointCloud::PointCloud(const string &path) {
    read(path);
}

void PointCloud::read(const string &path) {
    ifstream inf(path, ios::binary);

    if (inf.is_open()) {

        // header
        Header header;

        // fill in header ref with read data of size of header
        inf.read((char *) &header,
                 sizeof(header)); // cast to (char *) -> tell cpp we have some amount of bytes here/char array

        std::cout << "version " << +header.versionMaj << "." << +header.versionMin << std::endl;
        std::cout << "scale factors " << header.scaleX << " " << header.scaleY << " " << header.scaleZ << std::endl;

        // can only read las version 1.2
        assert(header.versionMaj == 1 && header.versionMin == 2);
        assert(header.headerSize == sizeof(header));
        assert(header.pointDataRecordFormat == 1 || header.pointDataRecordFormat == 2);
        pointRecFormat = header.pointDataRecordFormat;

        // var length records
        VarLenRecHeader varLenRecHeaders[header.numVarLenRecords]; // size two in this case
        GeoKeyDirectoryTag geoKeyDirectoryTag; // is required

        for (int i = 0; i < header.numVarLenRecords; i++) {

            // read header
            auto &currentHeader = varLenRecHeaders[i]; // ref
            inf.read((char *) &currentHeader, sizeof currentHeader);

            if (strcmp(currentHeader.userid, "LASF_Projection") == 0 && currentHeader.recordId == 34735) {
                //  this var length record is the GeoKeyDirectoryTag

                // read info
                inf.read((char *) &geoKeyDirectoryTag, 8);//sizeof geoKeyDirectoryTag);

                // resize entry vector of geo key directory tag
                geoKeyDirectoryTag.entries.resize(geoKeyDirectoryTag.wNumberOfKeys);
                // read entries
                inf.read((char *) &geoKeyDirectoryTag.entries[0],
                         geoKeyDirectoryTag.wNumberOfKeys * sizeof(GeoKeyEntry));
            }


        }




        // points
        std::cout << "num of points: " << header.numberOfPoints << std::endl;

        inf.seekg(header.pointDataOffset); // skip to point data

        // mittelpunkt von x, y, z - to center pointcloud
        auto midX = (header.maxX + header.minX) / 2.0f;
        auto midY = (header.maxY + header.minY) / 2.0f;
        auto midZ = (header.maxZ + header.minZ) / 2.0f;
        xOffset = (float) midX;
        yOffset = (float) midZ;
        zOffset = (float) midY;

        if (header.pointDataRecordFormat == 1) {
            for (uint32_t i = 0; i < header.numberOfPoints; i++) {
                PointDRF1 point;
                inf.read((char *) (&point), sizeof(PointDRF1));

                // convert to opengl friendly thing
                // Xcoordinate = (Xrecord * Xscale) + Xoffset

                // center pointcloud
                Vertex v;
                v.x = (float) (point.x * header.scaleX + header.offX - midX);
                v.y = (float) (point.z * header.scaleZ + header.offZ - midZ);
                v.z = -(float) (point.y * header.scaleY + header.offY - midY);

                //cout << v.x << ", " << v.y << ", " << v.z << endl;
                vertices.push_back(v);
            }
        } else if (header.pointDataRecordFormat == 2) {
            for (uint32_t i = 0; i < header.numberOfPoints; i++) {
                PointDRF2 point;
                inf.read((char *) (&point), sizeof(PointDRF2));

                // convert to opengl friendly thing
                // Xcoordinate = (Xrecord * Xscale) + Xoffset

                auto redInt = (point.red / 256);
                auto redFloat = (point.red / 65536.0f);
                auto greenFloat = (point.green / 65536.0f);
                auto blueFloat = (point.blue / 65536.0f);

                // center pointcloud
                ColorVertex v = {
                        (float) (point.x * header.scaleX + header.offX - midX),
                        (float) (point.z * header.scaleZ + header.offZ - midZ),
                        -(float) (point.y * header.scaleY + header.offY - midY),
                        redFloat, greenFloat, blueFloat

                };

                colorVertices.push_back(v);
            }
        }


        cout << "\t min\t max" << endl;

        cout << "x:\t" << header.minX - midX << "\t" << header.maxX - midX << endl;
        cout << "y:\t" << header.minY - midY << "\t" << header.maxY - midY << endl;
        cout << "z:\t" << header.minZ - midZ << "\t" << header.maxZ - midZ << endl;


        if (!inf.good())
            throw runtime_error("Reading LAS ran into error");

    } else {
        throw runtime_error("Cant find LAS file");
    }
}

uint32_t PointCloud::getVerticesCount() {
    if(hasColor())
        return (uint32_t) colorVertices.size();
    else
        return (uint32_t) vertices.size();
}

Vertex *PointCloud::getVertices() {
    return vertices.data();
}

Vertex *PointCloud::getColorVertices() {
    return colorVertices.data();
}

Vertex PointCloud::getUTMForOpenGL(Vertex *vertexOpenGL) {
    // TODO offset is float, losing precision
    return Vertex{vertexOpenGL->x + xOffset, vertexOpenGL->y + yOffset, vertexOpenGL->z + zOffset};
}

Vertex PointCloud::getWGSForOpenGL(Vertex *vertex) {
    // TODO offset is float, losing precision

    // wert in utm holen, dann:

    // zone number: 60 zones, each 6 degrees of longitude (horizontal stripes), number is consistent in horizontal stripes
    // zone letter: 20 zones, each 8 degrees of latitude (vertical stripes), letter is consistent in vertical stripes
    // x wert zwischen 100.000 und 899.999 meter in zone
    // y wert ist entfernugn vom äquator (zumindest auf nordhalbkugel)
    return Vertex();
}

bool PointCloud::hasColor() {
    return pointRecFormat == 2;
}
