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

        // can only read las version 1.2
        assert(header.versionMaj == 1 && header.versionMin == 2);
        assert(header.headerSize == sizeof(header));
        assert(header.pointDataRecordFormat == 1);


        inf.seekg(header.pointDataOffset); // skip to point data
        for(uint32_t i = 0; i < header.numberOfPoints; i++){

            PointDRF1 point;
            inf.read((char *)(&point), sizeof(PointDRF1));

            // convert to opengl friendly thing
            Vertex v = {
                    (float)point.x,
                    (float)point.y,
                    (float)point.z
                    };

            cout << v.x << ", " << v.y << ", " << v.z << endl;
            vertices.push_back(v);
        }

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