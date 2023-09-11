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
    Window(Vertex* vertices, uint32_t vertexCount);
//    void setVertices(Vertex* vertices);
private:
    int width;
    int height;
    std::string title;

    Vertex* vertices;
    uint32_t vertexCount;

    void processInput(GLFWwindow *window);
    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);

    void initGLFW();
    GLFWwindow* createWindow();

    void initGlew();

    // pass by ref
    void dataStuff(GLuint& VBO, GLuint& VAO);
};


#endif //MASTI_WINDOW_H
