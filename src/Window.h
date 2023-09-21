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
#include "shader.h"
#include "PointCloud.h"

#ifndef LASCAMPUS_WINDOW_H
#define LASCAMPUS_WINDOW_H


class Window {
public:
    Window(PointCloud pointCloud);

private:
    const int WIDTH;
    const int HEIGHT;
    const std::string TITLE;
    const float POINT_SIZE;

    // camera
    Camera camera = Camera(glm::vec3(0.0f, 10.0f, 3.0f));

    // mouse input
    bool firstMouse = true;
    float lastX = 800.0f / 2.0;
    float lastY = 600.0 / 2.0;

    // delta time for smooth movement, independent of render speed
    float deltaTime = 0.0f;    // Time between current frame and last frame
    float lastFrame = 0.0f; // Time of last frame

    // data
    Vertex *vertices;
    uint32_t vertexCount;

    bool showInfo = false;

    void processInput(GLFWwindow *window);

    static void framebuffer_size_callback(GLFWwindow *window, int width, int height);

    void mouse_callback(GLFWwindow *window);

//    static void scrollCallback(GLFWwindow* window, double xOffset, double yOffset);

    void initGLFW();

    GLFWwindow *createWindow();

    void initGlew();

    void shaderSettings(Shader &shader);

    // pass by ref
    void dataStuff(GLuint &VBO, GLuint &VAO, PointCloud pointCloud);

    void dataStuff2(GLuint &VBO, GLuint &VAO);

    Shader getPcShader(bool color);
};


#endif //LASCAMPUS_WINDOW_H