//
// Created by Jana Jansen on 01.09.23.
//

#include <iostream>
#include "Window.h"
#include "loadShader.h"

Window::Window(Vertex* vertices, uint32_t vertexCount) {

    //std::cout << vertexCount << " " << *vertices << std::endl;

    Window::vertices = vertices;

    // Initialise GLFW
    //glewExperimental = true; // Needed for core profile
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return;
    }

    glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
#endif
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL

    // Open a window and create its OpenGL context
    GLFWwindow* window = glfwCreateWindow(1024, 768, "Tutorial 01", NULL, NULL);
    if (window == NULL) {
        fprintf(stderr,
                "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n");
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // Initialize GLEW
    //glewExperimental=true; // Needed in core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        return;
    }

    // SHADERS
    // Create and compile our GLSL program from the shaders
    // normalerweise macht vertex shader die berechnung von iwelchen koords die man halt so hat zu koords in -1,1 (opengls visible region)
    GLuint programID = LoadShaders("/Users/Shared/Masti/LasCampus/src/SimpleVertexShader.vs",
                                   "/Users/Shared/Masti/LasCampus/src/SimpleFragmentShader.fs");

    // enable this -> set point size in vertex shader
    glEnable(GL_PROGRAM_POINT_SIZE);


    // DATIS


    GLuint VBO, VAO;
    glGenVertexArrays(1, &VAO);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(VAO);

    // Generate 1 buffer, put the resulting identifier in vbo
    glGenBuffers(1, &VBO);
    // jetzt wird der buffer gebindet und immer wenn wir jetzt calls zum GL_ARRAY_BUFFER target machen dann wird der aktuelle gebindete buffer verwendet
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    auto verticesByteSize = sizeof(std::vector<Vertex>) + (sizeof(Vertex) * vertexCount);
    //std::cout << "byte size" << verticesByteSize << std::endl;
    glBufferData(GL_ARRAY_BUFFER, verticesByteSize, vertices, GL_STATIC_DRAW);


    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // OPTIONAL: unbind
    // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    glBindVertexArray(0);


    // Ensure we can capture the escape key being pressed below
    //glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);




    do {
        // input
        processInput(window);

        // Clear the screen. It's not mentioned before Tutorial 02, but it can cause flickering, so it's there nonetheless.
        // glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);



        // Draw triangle...
        // Use our shader
        glUseProgram(programID);
        glBindVertexArray(VAO);

//        // Draw the triangle !
//        // GL_POINTS, GL_TRIANGLES
        glDrawArrays(GL_POINTS, 0, vertexCount); // Starting from vertex 0; 3 vertices total -> 1 triangle



        // front buffer showa frame, all rendering commands go to back buffer, swap when ready -> no flickering
        glfwSwapBuffers(window);
        // checks for events that where triggered -> calls functions that where registered via callbacks
        glfwPollEvents();

    } // Check if the ESC key was pressed or the window was closed
    while (!glfwWindowShouldClose(window));

    // optional: de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------------
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(programID);

    // Terminate GLFW
    glfwTerminate();
}

void Window::processInput(GLFWwindow *window)
{
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

//void Window::setVertices(Vertex* pVertices) {
//
//    vertices = pVertices;
//
//}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void Window::framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}
