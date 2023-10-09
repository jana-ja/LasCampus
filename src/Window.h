//
// Created by Jana Jansen on 01.09.23.
//

#include <stdio.h>
#include <stdlib.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "Vertex.h"

#ifndef MASTI_WINDOW_H
#define MASTI_WINDOW_H


class Window{
public:
    Window();
    void setVertices(Vertex* vertices);
private:
    Vertex* vertices;
};


#endif //MASTI_WINDOW_H
