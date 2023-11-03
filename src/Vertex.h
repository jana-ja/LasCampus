//
// Created by Jana Jansen on 01.09.23.
//

#ifndef LASCAMPUS_VERTEX_H
#define LASCAMPUS_VERTEX_H

struct Vertex {
    float x, y, z;//, nX, nY, nZ;

    float operator[](const int i) const{ // TODO macht das speicher größer?
        switch (i) {
            case 1:
                return x;
            case 2:
                return y;
            default:
                return z;
        }
    }
};

struct ColorVertex : Vertex {
    float x, y, z, red, green, blue;
};

#endif //LASCAMPUS_VERTEX_H
