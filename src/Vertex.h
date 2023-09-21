//
// Created by Jana Jansen on 01.09.23.
//

#ifndef LASCAMPUS_VERTEX_H
#define LASCAMPUS_VERTEX_H

struct Vertex {
    float x, y, z;
};

struct ColorVertex : Vertex {
    float x, red, y, green, z, blue;
};

#endif //LASCAMPUS_VERTEX_H
