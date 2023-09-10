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

void PointCloud::read(const string &path){
    ifstream inf(path, ios::binary);

    if(inf.is_open()){
        Header header;

        // fill in header ref with read data of size of header
        inf.read((char *)&header, sizeof(header)); // cast to (char *) -> tell cpp we have some amount of bytes here/char array

        std::cout << "version " << +header.versionMaj << "." << +header.versionMin << std::endl;
        std::cout << "scale factors " << header.scaleX << " " << header.scaleY << " " <<header.scaleZ << std::endl;

                                                                                     // can only read las version 1.2
        assert(header.versionMaj == 1 && header.versionMin == 2);
        assert(header.headerSize == sizeof(header));
        assert(header.pointDataRecordFormat == 1);

        std::cout << "num of points" << header.numberOfPoints << std::endl;

        inf.seekg(header.pointDataOffset); // skip to point data
        auto maxV = std::max(header.maxX, header.maxY);
        std::cout << "maxes: x " << header.maxX << " y " << header.maxY << " z " << header.maxZ << std::endl;
        maxV = std::max(maxV, header.scaleZ);
        std::cout << "maxV: " << maxV << std::endl;
        for(uint32_t i = 0; i < header.numberOfPoints; i++){



            PointDRF1 point;
            inf.read((char *)(&point), sizeof(PointDRF1));

            // convert to opengl friendly thing
            // Xcoordinate = (Xrecord * Xscale) + Xoffset
            // größtes maxV suchen und alle dadurch teilen?

            // mein z = deren -x
            // mein x deren y
            // mein y deren z

            Vertex v = {
                    (float)(point.x * header.scaleX + header.offX),// / (float)header.maxX,
                    (float)(point.y * header.scaleY + header.offY),// / (float)header.maxX,
                    (float)(point.z * header.scaleZ + header.offZ)// / (float)header.maxX
                    };

            //cout << v.x << ", " << v.y << ", " << v.z << endl;
            vertices.push_back(v);
        }

        auto itX = minmax_element(
                std::begin(vertices),
                std::end(vertices),
                [](const Vertex& a,const Vertex& b) { return a.x < b.x; });
        auto minX = (*itX.first).x;
        auto maxX = (*itX.second).x;
        cout << "min x: " << minX << " min x header: " << header.minX << " max X: " << maxX << " max x header: " << header.maxX  << endl;

        auto ity = minmax_element(
                std::begin(vertices),
                std::end(vertices),
                [](const Vertex& a,const Vertex& b) { return a.y < b.y; });
        auto miny = (*ity.first).y;
        auto maxy = (*ity.second).y;
        cout << "min y: " << miny << " min y header: " << header.minY << " max y: " << maxy << " max y header: " << header.maxY << endl;

        auto itz = minmax_element(
                std::begin(vertices),
                std::end(vertices),
                [](const Vertex& a,const Vertex& b) { return a.z < b.z; });
        auto minz = (*itz.first).z;
        auto maxz = (*itz.second).z;
        cout << "min z: " << minz << " min z header: " << header.minZ << " max z: " << maxz << " max z header: " << header.maxZ << endl;

        if(!inf.good())
            throw runtime_error("Reading LAS ran into error");

    } else {
        throw runtime_error("Cant find LAS file");
    }
}

uint32_t PointCloud::getVerticesCount(){
    return (uint32_t)vertices.size();
}

Vertex* PointCloud::getVertices(){
    return vertices.data();
}