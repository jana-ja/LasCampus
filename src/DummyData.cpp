//
// Created by Jana Jansen on 01.09.23.
//


#include <vector>
#include <iostream>
#include "DummyData.h"
DummyData::DummyData() {
    for (int i = 0; i < 100; ++i) {
        Vertex v = {
                (float)(rand()) / (float)(RAND_MAX),
                (float)(rand()) / (float)(RAND_MAX),
                1.0f
        };
        vertices.push_back(v);

        std::cout << v.x << ", " << v.y << ", " << v.z << std::endl;
    }
}

uint32_t DummyData::getVerticesCount(){
    return (uint32_t)vertices.size();
}


// TODO pass by reference ansehen
Vertex* DummyData::getVertices(){
    return vertices.data();
}

