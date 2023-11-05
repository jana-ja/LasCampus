//
// Created by Jana Jansen on 04.11.23.
//

#ifndef LASCAMPUS_POINTCLOUDDATASTRUCTURE_H
#define LASCAMPUS_POINTCLOUDDATASTRUCTURE_H

#include <vector>
#include "Vertex.h"

struct PointCloudNode {
public:
    virtual Vertex* getVertex() = 0;

private:
    Vertex* vertex;

};

class PointCloudDataStructure {
public:
    PointCloudDataStructure(const std::vector<Vertex>* points);

    virtual double kNN(const Vertex& point, size_t k,
                       std::vector<PointCloudNode>* result) = 0;
    virtual uint32_t getPointCount() = 0;
    virtual PointCloudNode* getPoints() = 0;

private:

    double length;      // Length of a box
    double breadth;     // Breadth of a box
    double height;      // Height of a box
};





#endif //LASCAMPUS_POINTCLOUDDATASTRUCTURE_H
