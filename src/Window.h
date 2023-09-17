//
// Created by Jana Jansen on 01.09.23.
//

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <GL/glew.h> // Always include it before gl.h and glfw3.h, since it's a bit magic.
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Camera.h"

#ifndef LASCAMPUS_WINDOW_H
#define LASCAMPUS_WINDOW_H


class Window {
public:
    Window(Vertex *vertices, uint32_t vertexCount);
//    void setVertices(Vertex* vertices);
    void mouse_callback(GLFWwindow* window);

private:
    const int WIDTH;
    const int HEIGHT;
    const std::string TITLE;
    const float POINT_SIZE;

    // camera pos
    Camera camera = Camera(glm::vec3(0.0f, 0.0f, 3.0f));

    // camera angle
    bool firstMouse = true;
    float lastX =  800.0f / 2.0;
    float lastY =  600.0 / 2.0;

    // delta time for smooth movement, independent of render speed
    float deltaTime = 0.0f;	// Time between current frame and last frame
    float lastFrame = 0.0f; // Time of last frame

    Vertex *vertices;
    uint32_t vertexCount;

    void processInput(GLFWwindow *window);


    static void framebuffer_size_callback(GLFWwindow *window, int width, int height);

    void initGLFW();

    GLFWwindow *createWindow();

    void initGlew();

    // pass by ref
    void dataStuff(GLuint &VBO, GLuint &VAO);

    void staticShaderSettings(GLuint shaderPID);

    void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

};


#endif //LASCAMPUS_WINDOW_H
