//
// Created by Jana Jansen on 01.09.23.
//

#ifndef MASTI_DUMMYDATA_H
#define MASTI_DUMMYDATA_H

#include "Vertex.h"
class DummyData {
public:
    DummyData();

    uint32_t getVerticesCount();

    Vertex *getVertices();

private:
    std::vector<Vertex> vertices;
};

#endif //MASTI_DUMMYDATA_H