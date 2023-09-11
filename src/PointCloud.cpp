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

        std::cout << "num of points: " << header.numberOfPoints << std::endl;

        inf.seekg(header.pointDataOffset); // skip to point data

        // mittelpunkt von x, y, z - to center pointcloud
        auto midX = (header.maxX + header.minX) / 2.0f;
        auto midY = (header.maxY + header.minY) / 2.0f;
        auto midZ = (header.maxZ + header.minZ) / 2.0f;

        //auto maxV = 100.0;//max(header.maxX - midX, max(header.maxY - midY, header.maxZ - midZ));

        for(uint32_t i = 0; i < header.numberOfPoints; i++){
            PointDRF1 point;
            inf.read((char *)(&point), sizeof(PointDRF1));

            // convert to opengl friendly thing
            // Xcoordinate = (Xrecord * Xscale) + Xoffset

            // mein z = deren -x
            // mein x deren y
            // mein y deren z

            // center pointcloud
            Vertex v = {
                    (float)(point.x * header.scaleX + header.offX - midX),
                    (float)(point.y * header.scaleY + header.offY - midY),
                    (float)(point.z * header.scaleZ + header.offZ - midZ)
                    };

            //cout << v.x << ", " << v.y << ", " << v.z << endl;
            vertices.push_back(v);
        }


        cout << "\t min\t max" << endl;

        cout << "x:\t" << header.minX - midX << "\t" << header.maxX - midX  << endl;
        cout << "y:\t" << header.minY - midY << "\t" << header.maxY - midY  << endl;
        cout << "z:\t" << header.minZ - midZ << "\t" << header.maxZ - midZ  << endl;




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