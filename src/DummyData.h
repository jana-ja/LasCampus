//
// Created by Jana Jansen on 01.09.23.
//

#ifndef LASCAMPUS_DUMMYDATA_H
#define LASCAMPUS_DUMMYDATA_H

#include "Vertex.h"
class DummyData {
public:
    DummyData();

    uint32_t getVerticesCount();

    Vertex *getVertices();

private:
    std::vector<Vertex> vertices;
};

#endif //LASCAMPUS_DUMMYDATA_H