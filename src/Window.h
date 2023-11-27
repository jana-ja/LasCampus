//
// Created by Jana Jansen on 01.09.23.
//

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <GL/glew.h> // Always include it before gl.h and glfw3.h, since it's a bit magic.
#include <GLFW/glfw3.h>
#include "glm/glm.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Camera.h"
#include "shader.h"
#include "DataStructure.h"

#ifndef LASCAMPUS_WINDOW_H
#define LASCAMPUS_WINDOW_H


class Window {
public:
    Window(DataStructure pointCloud);

private:
    const char* TAG = "Window\t";

    const int WIDTH;
    const int HEIGHT;
    const std::string TITLE;
    const float POINT_SIZE;

    bool f1Pressed = false;

    // camera
    Camera camera = Camera(glm::vec3(0.0f, 10.0f, 0.0f));//Camera(glm::vec3(0.0f, 10.0f, 3.0f));
//    Camera camera = Camera(glm::vec3(389500.0f, 131.0f, 5705500.0f));//Camera(glm::vec3(0.0f, 10.0f, 3.0f));

    // mouse input
    bool firstMouse = true;
    float lastX = 800.0f / 2.0;
    float lastY = 600.0 / 2.0;

    // delta time for smooth movement, independent of render speed
    float deltaTime = 0.0f;    // Time between current frame and last frame
    float lastFrame = 0.0f; // Time of last frame

    // tree
    DataStructure pointCloud;

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
    void dataStuffPointCloud(GLuint &VBO, GLuint &VAO, DataStructure pointCloud);
    void dataStuffCoordSys(GLuint &VBO, GLuint &VAO);
    void dataStuffNormals(GLuint &VBO, GLuint &VAO, DataStructure pointCloud);

};


#endif //LASCAMPUS_WINDOW_H