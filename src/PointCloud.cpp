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
        assert(header.pointDataRecordFormat == 1);


        // var length records
        VarLenRecHeader varLenRecHeaders[header.numVarLenRecords]; // size two in this case
        GeoKeyDirectoryTag geoKeyDirectoryTag; // is required

        for(int i = 0; i < header.numVarLenRecords; i++){
//            auto testi = varLenRecHeaders[i]; // testi and varLenRecHeaders[0] have different addresses
            // &varLenRecHeaders[i] == currentHeaderPtr == &(*currentHeaderPtr)  !=  &testi

            // read header
            auto currentHeaderPtr = &varLenRecHeaders[i];
            auto he = *currentHeaderPtr;
            inf.read((char *) &he, sizeof he);

            if(strcmp(he.userid, "LASF_Projection") == 0  && he.recordId == 34735){
                //  this var length record is the GeoKeyDirectoryTag
                inf.read((char *) &geoKeyDirectoryTag, sizeof geoKeyDirectoryTag);
                // geo key entries
                GeoKeyEntry geoKeyEntries[geoKeyDirectoryTag.wNumberOfKeys];
                std::cout << "record len after ehader: " << he.recordLenAfterHeader << " size of struct " << sizeof geoKeyDirectoryTag + geoKeyDirectoryTag.wNumberOfKeys * sizeof(GeoKeyEntry) << std::endl;
                inf.read((char *) &geoKeyEntries, geoKeyDirectoryTag.wNumberOfKeys * sizeof(GeoKeyEntry));
                std::cout << "lol " << geoKeyEntries[0].wCount << std::endl;
            }


        }

        // TODO assert GeoKeyDirectoryTag is there



        // points
        std::cout << "num of points: " << header.numberOfPoints << std::endl;

        inf.seekg(header.pointDataOffset); // skip to point data

        // mittelpunkt von x, y, z - to center pointcloud
        auto midX = (header.maxX + header.minX) / 2.0f;
        auto midY = (header.maxY + header.minY) / 2.0f;
        auto midZ = (header.maxZ + header.minZ) / 2.0f;

        for (uint32_t i = 0; i < header.numberOfPoints; i++) {
            PointDRF1 point;
            inf.read((char *) (&point), sizeof(PointDRF1));

            // convert to opengl friendly thing
            // Xcoordinate = (Xrecord * Xscale) + Xoffset

            // center pointcloud
            Vertex v = {
                    (float) (point.x * header.scaleX + header.offX - midX),
                    (float) (point.z * header.scaleZ + header.offZ - midZ),
                    -(float) (point.y * header.scaleY + header.offY - midY)

            };

            //cout << v.x << ", " << v.y << ", " << v.z << endl;
            vertices.push_back(v);
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
    return (uint32_t) vertices.size();
}

Vertex *PointCloud::getVertices() {
    return vertices.data();
}