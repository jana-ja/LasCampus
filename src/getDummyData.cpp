//
// Created by Jana Jansen on 01.09.23.
//


#include <vector>
#include <iostream>
#include "getDummyData.h"

Vertex *getDummyVertices() {
    std::vector<Vertex> vertices;

    for (int i = 0; i < 100; ++i) {
        Vertex v = {
                (float) (rand() % 100),
                (float) (rand() % 100),
                (float) (rand() % 100)
        };
        vertices.push_back(v);

        // std::cout << v.x << ", " << v.y << ", " << v.z << std::endl;
    }

    return nullptr;
}

