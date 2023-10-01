//
// Created by Jana Jansen on 01.09.23.
//


#include "Vertex.h"
#include <stdexcept>
#include "Window.h"


Window::Window(PointCloud pointCloud) : WIDTH(1024), HEIGHT(768), TITLE("Campus"), POINT_SIZE(10.0f),
                                        pointCloud(pointCloud) {


    // glfw
    initGLFW();
    GLFWwindow* window = createWindow();


    // glew
    initGlew();


    // point cloud
    // shader
    Shader pcShader = getPcShader(pointCloud.hasColor());
    shaderSettings(pcShader);
    // data
    GLuint pcVBO, pcVAO;
    dataStuff(pcVBO, pcVAO, pointCloud);

    // coordinate system
    // shader
    Shader csShader("/Users/Shared/Masti/LasCampus/src/CoordSysVertexShader.vs",
                    "/Users/Shared/Masti/LasCampus/src/CoordSysFragmentShader.fs");
    shaderSettings(csShader);
    // data
    GLuint csVBO, csVAO;
    dataStuff2(csVBO, csVAO);


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


        pcShader.use();
        // transforms: camera - view space
        glm::mat4 view = camera.GetViewMatrix();
        pcShader.setMat4("view", view);
        // update cameraPos in pcShader for dynamic point size
        pcShader.setVec3("cameraPos", camera.position);


        // draw point cloud
        glBindVertexArray(pcVAO);
        glDrawArrays(GL_POINTS, 0, pointCloud.getVertexCount()); // Starting from vertex 0


        if (showInfo) {
            csShader.use();
            // transforms: camera - view space
            glm::mat4 view = camera.GetViewMatrix();
            csShader.setMat4("view", view);


            // draw coordinate sys lines
            glBindVertexArray(csVAO);
            auto pos = glm::vec3(camera.position + camera.front);
            glm::mat4 model = glm::mat4(1.0f);
            model = glm::translate(model, pos);
            csShader.setMat4("model", model);
            glDrawArrays(GL_LINES, 0, 6);
        }


        // front buffer shows frame, all rendering commands go to back buffer, swap when ready -> no flickering
        glfwSwapBuffers(window);
        // checks for events that where triggered -> calls functions that where registered via callbacks
        glfwPollEvents();
        // call this here instead of setting a callback function, because callback function needs to be static/global
        this->mouse_callback(window);

    } // check if window should close (close button or esc key)
    while (!glfwWindowShouldClose(window));

    // optional: de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------------
    glDeleteVertexArrays(1, &pcVAO);
    glDeleteBuffers(1, &pcVBO);
    glDeleteVertexArrays(1, &csVAO);
    glDeleteBuffers(1, &csVBO);
    glDeleteProgram(csShader.ID);
    glDeleteProgram(pcShader.ID);

    // Terminate GLFW
    glfwTerminate();
}

void Window::processInput(GLFWwindow *window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_F1) == GLFW_PRESS)
        showInfo = !showInfo;

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        camera.ProcessKeyboard(UP, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
        camera.ProcessKeyboard(DOWN, deltaTime);
}


// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void Window::framebuffer_size_callback(GLFWwindow *window, int width, int height) {
    // make sure the viewport matches the new window dimensions; note that WIDTH and
    // HEIGHT will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

void Window::mouse_callback(GLFWwindow *window) {
    GLdouble xPos, yPos;
    glfwGetCursorPos(window, &xPos, &yPos);

    if (firstMouse) {
        lastX = xPos;
        lastY = yPos;
        firstMouse = false;
    }

    float xoffset = xPos - lastX;
    float yoffset = lastY - yPos;
    lastX = xPos;
    lastY = yPos;

    camera.ProcessMouseMovement(xoffset, yoffset);

}

//void Window::scrollCallback(GLFWwindow* window, double xOffset, double yOffset)
//{
//    camera.ProcessMouseScroll(static_cast<float>(yOffset));
//}

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
    //glfwSetCursorPosCallback(window, static_mouse_callback(this));
//    glfwSetScrollCallback(window, scrollCallback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    return window;
}

void Window::initGlew() {
    //glewExperimental=true; // Needed in core profile
    if (glewInit() != GLEW_OK) {
        throw std::runtime_error("Failed to initialize GLEW.");
    }

    glEnable(GL_PROGRAM_POINT_SIZE); // enable this -> set point size in vertex shader
    glEnable(GL_DEPTH_TEST); // 3D graphics
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(5.0f);
}


void Window::shaderSettings(Shader &shader) {
    shader.use();
    glm::mat4 projection = glm::perspective(glm::radians(camera.zoom), (float) WIDTH / (float) HEIGHT, 0.1f, 100.0f);
    shader.setMat4("projection", projection);
    shader.setFloat("pointSize", POINT_SIZE);
}

void Window::dataStuff(GLuint &VBO, GLuint &VAO, PointCloud pointCloud) {

    glGenVertexArrays(1, &VAO);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(VAO);

    // Generate 1 buffer, put the resulting identifier in vbo
    glGenBuffers(1, &VBO);
    // jetzt wird der buffer gebindet und immer wenn wir jetzt calls zum GL_ARRAY_BUFFER target machen dann wird der aktuelle gebindete buffer verwendet
    glBindBuffer(GL_ARRAY_BUFFER, VBO);


    if (pointCloud.hasColor()) {
        auto verticesByteSize = (sizeof(ColorVertex) * pointCloud.getVertexCount());
//        auto verticesByteSize = pointCloud.getVerticesSize();
//        sizeof(MyVector) + (sizeof(MyVector[0]) * MyVector.size())
        glBufferData(GL_ARRAY_BUFFER, verticesByteSize, pointCloud.getColorVertices(), GL_STATIC_DRAW);
        // position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *) 0);
        glEnableVertexAttribArray(0);
        // color attribute // später richtige farben/texture
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *) (3 * sizeof(float)));
        glEnableVertexAttribArray(1);
    } else {
        auto verticesByteSize = (sizeof(Vertex) * pointCloud.getVertexCount());
        glBufferData(GL_ARRAY_BUFFER, verticesByteSize, pointCloud.getVertices(), GL_STATIC_DRAW);
        // position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *) 0);
        glEnableVertexAttribArray(0);
    }


    // OPTIONAL: unbind
    // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    glBindVertexArray(0);
}

void Window::dataStuff2(GLuint &VBO, GLuint &VAO) {

    glGenVertexArrays(1, &VAO);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(VAO);

    // Generate 1 buffer, put the resulting identifier in vbo
    glGenBuffers(1, &VBO);
    // jetzt wird der buffer gebindet und immer wenn wir jetzt calls zum GL_ARRAY_BUFFER target machen dann wird der aktuelle gebindete buffer verwendet
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    float lineLength = 0.2f;
    float coordPoints[] = {
            // positions                      // colors
            0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, // center
            lineLength, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, // x
            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, // center
            0.0f, lineLength, 0.0f, 0.0f, 1.0f, 0.0f, // y
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, // center
            0.0f, 0.0f, lineLength, 0.0f, 0.0f, 1.0f // z
    };
    glBufferData(GL_ARRAY_BUFFER, sizeof(coordPoints), coordPoints, GL_STATIC_DRAW);


    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *) 0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *) (3 * sizeof(float)));
    glEnableVertexAttribArray(1);


    // OPTIONAL: unbind
    // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    glBindVertexArray(0);
}

Shader Window::getPcShader(bool hasColor) {
    if (hasColor) {
        return Shader("/Users/Shared/Masti/LasCampus/src/PointCloudColorVertexShader.vs",
                      "/Users/Shared/Masti/LasCampus/src/PointCloudColorFragmentShader.fs");
    } else {
        return Shader("/Users/Shared/Masti/LasCampus/src/PointCloudVertexShader.vs",
                      "/Users/Shared/Masti/LasCampus/src/PointCloudFragmentShader.fs");
    }
}
