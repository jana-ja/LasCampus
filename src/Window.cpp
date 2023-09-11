//
// Created by Jana Jansen on 01.09.23.
//


#include "Vertex.h"
#include <stdexcept>
#include "Window.h"
#include "loadShader.h"


Window::Window(Vertex *vertices, uint32_t vertexCount) : WIDTH(1024), HEIGHT(768), TITLE("Campus"), POINT_SIZE(10.0f), vertices(vertices),
                                                         vertexCount(vertexCount) {

    // glfw
    initGLFW();
    GLFWwindow *window = createWindow();

    // glew
    initGlew();
    GLuint shaderPID = LoadShaders("/Users/Shared/Masti/LasCampus/src/SimpleVertexShader.vs",
                                   "/Users/Shared/Masti/LasCampus/src/SimpleFragmentShader.fs");

    staticShaderSettings(shaderPID);



    glEnable(GL_PROGRAM_POINT_SIZE); // enable this -> set point size in vertex shader
    glEnable(GL_DEPTH_TEST);


    // data
    GLuint VBO, VAO;
    dataStuff(VBO, VAO);


    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);


    // render loop
    do {
        // per-frame time logic
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        processInput(window);

        // Clear the screen.
        // glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


//        // Use shader
//        glUseProgram(shaderPID); // muss das in loop sein??
        // transforms: camera - view space
        glm::mat4 view  = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp); // initial values are set in header
        // retrieve the matrix uniform locations
        unsigned int viewLoc = glGetUniformLocation(shaderPID, "view");
        unsigned int cameraLoc = glGetUniformLocation(shaderPID, "cameraPos");
        // pass them to the shaders
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, &view[0][0]);
        glUniform3f(cameraLoc, cameraPos.x, cameraPos.y, cameraPos.z);



        // draw point cloud
        glBindVertexArray(VAO);
        glDrawArrays(GL_POINTS, 0, vertexCount); // Starting from vertex 0


        // front buffer shows frame, all rendering commands go to back buffer, swap when ready -> no flickering
        glfwSwapBuffers(window);
        // checks for events that where triggered -> calls functions that where registered via callbacks
        glfwPollEvents();

    } // check if window should close (close button or esc key)
    while (!glfwWindowShouldClose(window));

    // optional: de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------------
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderPID);

    // Terminate GLFW
    glfwTerminate();
}

void Window::processInput(GLFWwindow *window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    float cameraSpeed = static_cast<float>(CAMERA_SPEED * deltaTime);
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        cameraPos += cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraPos -= cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
}

//void Window::setVertices(Vertex* pVertices) {
//
//    vertices = pVertices;
//
//}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void Window::framebuffer_size_callback(GLFWwindow *window, int width, int height) {
    // make sure the viewport matches the new window dimensions; note that WIDTH and
    // HEIGHT will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

void Window::initGLFW() {
    //glewExperimental = true; // Needed for core profile
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
#endif
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL
}

GLFWwindow *Window::createWindow() {
    // Open a window and create its OpenGL context
    GLFWwindow *window = glfwCreateWindow(WIDTH, HEIGHT, TITLE.c_str(), NULL, NULL);
    if (window == NULL) {
        glfwTerminate();
        throw std::runtime_error("Failed to open GLFW window.");
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    return window;
}

void Window::initGlew() {
    //glewExperimental=true; // Needed in core profile
    if (glewInit() != GLEW_OK) {
        throw std::runtime_error("Failed to initialize GLEW.");
    }
}

void Window::dataStuff(GLuint &VBO, GLuint &VAO) {

    glGenVertexArrays(1, &VAO);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(VAO);

    // Generate 1 buffer, put the resulting identifier in vbo
    glGenBuffers(1, &VBO);
    // jetzt wird der buffer gebindet und immer wenn wir jetzt calls zum GL_ARRAY_BUFFER target machen dann wird der aktuelle gebindete buffer verwendet
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    auto verticesByteSize = /*sizeof(std::vector<Vertex>) +*/ (sizeof(Vertex) *
                                                               vertexCount); // nur (sizeof(Vertex) * vertexCount) ??
    //std::cout << "byte size" << verticesByteSize << std::endl;
    glBufferData(GL_ARRAY_BUFFER, verticesByteSize, vertices, GL_STATIC_DRAW);


    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *) 0);
    glEnableVertexAttribArray(0);

    // OPTIONAL: unbind
    // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    glBindVertexArray(0);
}

void Window::staticShaderSettings(GLuint shaderPID) {
    // Use shader
    glUseProgram(shaderPID);
    glm::mat4 projection = glm::mat4(1.0f);
    projection = glm::perspective(glm::radians(45.0f), (float) WIDTH / (float) HEIGHT, 0.1f, 100.0f);
    unsigned int projectionLoc = glGetUniformLocation(shaderPID, "projection");
    glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));
    unsigned int pointSize = glGetUniformLocation(shaderPID, "pointSize");
    glUniform1f(pointSize, POINT_SIZE);
}


