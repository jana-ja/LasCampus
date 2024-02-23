//
// Created by Jana Jansen on 01.09.23.
//


#include "Window.h"
#include <stdexcept>
#include <vector>
#include "util.h"


Window::Window(DataStructure& pointCloud) : WIDTH(1024), HEIGHT(768), TITLE("Campus"), POINT_SIZE(10.0f),
                                           pointCloud(pointCloud){

    // set camera pos
//    camera.position = glm::vec3(pointCloud.xOffset, pointCloud.yOffset + 10, -pointCloud.zOffset);


    // glfw
    initGLFW();
    GLFWwindow *window = createWindow();
    std::cout << TAG << "Init glfw successful" << std::endl;


    // glew
    initGlew();
    std::cout << TAG << "Init glew successful" << std::endl;


    // point cloud
    // shader
    Shader pointShader = Shader("../src/shader/PointCloudVertexShader.vs", "../src/shader/PointCloudFragmentShader.fs");
    shaderSettings(pointShader);
    // colors / lighting
    pointShader.setVec3("light_color", 1.0f, 1.0f, 1.0f);
    pointShader.setVec3("light_pos", 0.0f, 100.0f, 0.0f); // PointCloudShader

    Shader splatShader = Shader("../src/shader/EllipticalSplatVertexShader.vs", "../src/shader/EllipticalSplatFragmentShader.fs");
    shaderSettings(splatShader);
        // colors / lighting
        splatShader.setVec3("light_dir", -0.25f, -0.5f, 0.25f); // south-west
        float viewport[4];
        glGetFloatv(GL_VIEWPORT, viewport);
        float wv = viewport[2];
        float hv = viewport[3];
        float hn = 2.0 * Z_NEAR * tan(45.0*M_PI/360.0);
        float wn = hn/hv*wv;
        float size_const = 2.0 * Z_NEAR * hv / hn;
        glm::vec4  vp(wn/wv, hn/hv, -0.5*wn,  -0.5*hn);
        glm::vec3  zb(Z_NEAR/(Z_NEAR-Z_FAR),	1.0/(Z_NEAR-Z_FAR), -Z_NEAR);
        splatShader.setFloat("size_const", size_const);
        splatShader.setVec4("vp", vp);
        splatShader.setVec3("zb", zb);

    // texture
    unsigned int texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    // set the texture wrapping/filtering options (on the currently bound texture object)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // read image
    int width, height;
    const int desiredChannels = 3;
    std::string imgDir = ".." + Util::PATH_SEPARATOR + "img" + Util::PATH_SEPARATOR;
    std::string imgPath = imgDir + pointCloud.getImgFile();
    // if file has less then desired channels, remaining fields will be 255
    int actualChannels;
    unsigned char* data = stbi_load(imgPath.c_str(), &width, &height, &actualChannels, desiredChannels);
    if (data != nullptr) {
        // texture target, mipmap level, color format for texture, width, height, always zero, color format of source image, datatype of source (here char/bytes), image data
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    } else {
        std::cout << TAG << "Failed to load texture" << std::endl;
    }
    stbi_image_free(data);

    // data
    GLuint pcVBO, t1VBO, t2VBO, texCoordVBO, pcVAO;
    dataStuffPointCloud(pcVBO, t1VBO, t2VBO, texCoordVBO, pcVAO, pointCloud);

    // coordinate system
    // shader
    Shader csShader("../src/shader/CoordSysVertexShader.vs",
                    "../src/shader/CoordSysFragmentShader.fs");
    shaderSettings(csShader);
    // data
    GLuint csVBO, csVAO;
    dataStuffCoordSys(csVBO, csVAO);

    // normals
    // shader
    Shader normalsShader("../src/shader/NormalsVertexShader.vs",
                         "../src/shader/NormalsFragmentShader.fs");
    shaderSettings(normalsShader);
    // data
    GLuint normalsVBO, normalsVAO;
    dataStuffNormals(normalsVBO, normalsVAO, pointCloud);


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



        if(useSplatShader) {
            splatShader.use();
            // transforms: camera - view space
            glm::mat4 view = camera.GetViewMatrix();
            splatShader.setMat4("view_matrix", view);
            splatShader.setBool("backface_culling", backfaceCulling);
            splatShader.setBool("enable_textures", enableTextures);
        } else {
            pointShader.use();
            // transforms: camera - view space
            glm::mat4 view = camera.GetViewMatrix();
            pointShader.setMat4("view_matrix", view);
            // update cameraPos in pointShader for dynamic point size
            pointShader.setVec3("camera_pos", camera.position);
            pointShader.setBool("enable_textures", enableTextures);
        }


        // draw point cloud
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture); //TODO nach oben schieben weil immer gleich?
        glBindVertexArray(pcVAO);
        glDrawArrays(GL_POINTS, 0, pointCloud.getVertexCount()); // Starting from vertex 0


        if (showInfo) {
            // coordinate system
            csShader.use();
            glLineWidth(2.0f);
            // transforms: camera - view space
            glm::mat4 view = camera.GetViewMatrix();
            csShader.setMat4("view_matrix", view);

            // draw coordinate sys lines
            glBindVertexArray(csVAO);
            auto pos = glm::vec3(camera.position + camera.front);
            glm::mat4 model = glm::mat4(1.0f);
            model = glm::translate(model, pos);
            csShader.setMat4("model_matrix", model);
            glDrawArrays(GL_LINES, 0, 6);

            // normals
            glLineWidth(0.5f);
            normalsShader.use();
            // transforms: camera - view space
            normalsShader.setMat4("view_matrix", view);
            // draw normals
            glBindVertexArray(normalsVAO);
            glDrawArrays(GL_LINES, 0, 2 * pointCloud.getVertexCount());
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
    glDeleteProgram(pointShader.ID);

    // Terminate GLFW
    glfwTerminate();
}

void Window::processInput(GLFWwindow *window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_F1) == GLFW_PRESS)
        f1Pressed = true;
    if (glfwGetKey(window, GLFW_KEY_F1) == GLFW_RELEASE && f1Pressed){
        f1Pressed = false;
        showInfo = !showInfo;
    }

    if (glfwGetKey(window, GLFW_KEY_F2) == GLFW_PRESS)
        f2Pressed = true;
    if (glfwGetKey(window, GLFW_KEY_F2) == GLFW_RELEASE && f2Pressed){
        f2Pressed = false;
        useSplatShader = !useSplatShader;
    }

    if (glfwGetKey(window, GLFW_KEY_F3) == GLFW_PRESS)
        f3Pressed = true;
    if (glfwGetKey(window, GLFW_KEY_F3) == GLFW_RELEASE && f3Pressed){
        f3Pressed = false;
        backfaceCulling = !backfaceCulling;
    }

    // f4 - enable textures
    if (glfwGetKey(window, GLFW_KEY_F4) == GLFW_PRESS)
        f4Pressed = true;
    if (glfwGetKey(window, GLFW_KEY_F4) == GLFW_RELEASE && f4Pressed){
        f4Pressed = false;
        enableTextures = !enableTextures;
    }


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
    glm::mat4 projection = glm::perspective(glm::radians(camera.zoom), (float) WIDTH / (float) HEIGHT, Z_NEAR, Z_FAR);
    shader.setMat4("projection_matrix", projection);
    shader.setFloat("point_size", POINT_SIZE); // TODO weg?
}

void Window::dataStuffPointCloud(GLuint &VBO, GLuint& t1VBO, GLuint& t2VBO, GLuint& texCoordVBO, GLuint& VAO, DataStructure& pointCloud) {
    glGenVertexArrays(1, &VAO);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(VAO);

    // Generate 1 buffer, put the resulting identifier in vbo
    glGenBuffers(1, &VBO);
    // jetzt wird der buffer gebindet und immer wenn wir jetzt calls zum GL_ARRAY_BUFFER target machen dann wird der aktuelle gebindete buffer verwendet
    glBindBuffer(GL_ARRAY_BUFFER, VBO);


    auto verticesByteSize = (sizeof(pcl::PointNormal) * pointCloud.getVertexCount());
    // sizeof(PointNormal) = 48 -> 12 floats. 4 for point, 4 for normal, 4 for curvature
    glBufferData(GL_ARRAY_BUFFER, verticesByteSize, pointCloud.getVertices(), GL_STATIC_DRAW);
    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(pcl::PointNormal), (void *) 0);
        // alt: (0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *) 0) and use vec4 in shader, because PointXYZ has 4 floats internally
    glEnableVertexAttribArray(0);
    // normal attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(pcl::PointNormal), (void *) (offsetof(pcl::PointXYZRGBNormal, normal)));
    glEnableVertexAttribArray(1);
    // color attribute
    glVertexAttribPointer(2, 3, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(pcl::PointNormal), (void *) (8 * sizeof(float))); // durch GL_TRUE wird unsigned byte richtig erkannnt und zu float konvertiert
    glEnableVertexAttribArray(2);
    // radius attribute
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(pcl::PointNormal), (void *) (offsetof(pcl::PointXYZRGBNormal, curvature)));
    glEnableVertexAttribArray(3);

    // tangent 1
    glGenBuffers(1, &t1VBO);
    glBindBuffer(GL_ARRAY_BUFFER, t1VBO);
    glBufferData(GL_ARRAY_BUFFER, (sizeof(pcl::PointXYZ) * pointCloud.getVertexCount()), pointCloud.getTangent1Vec(), GL_STATIC_DRAW);
    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(pcl::PointXYZ), (void *) 0);
    glEnableVertexAttribArray(4);
    // tangent 2
    glGenBuffers(1, &t2VBO);
    glBindBuffer(GL_ARRAY_BUFFER, t2VBO);
    glBufferData(GL_ARRAY_BUFFER, (sizeof(pcl::PointXYZ) * pointCloud.getVertexCount()), pointCloud.getTangent2Vec(), GL_STATIC_DRAW);
    glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, sizeof(pcl::PointXYZ), (void *) 0);
    glEnableVertexAttribArray(5);

    // texture coords
    glGenBuffers(1, &texCoordVBO);
    glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO);
    glBufferData(GL_ARRAY_BUFFER, (sizeof(pcl::PointXY) * pointCloud.getVertexCount()), pointCloud.getTexCoords(), GL_STATIC_DRAW);
    // index, size, type, normalized, stride, offset
    glVertexAttribPointer(6, 2, GL_FLOAT, GL_FALSE, sizeof(pcl::PointXY), (void *) 0);
    glEnableVertexAttribArray(6);

    // OPTIONAL: unbind
    // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    glBindVertexArray(0);
}


void Window::dataStuffNormals(GLuint &VBO, GLuint &VAO, DataStructure& pointCloud) {
    glGenVertexArrays(1, &VAO);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(VAO);

    // Generate 1 buffer, put the resulting identifier in vbo
    glGenBuffers(1, &VBO);
    // jetzt wird der buffer gebindet und immer wenn wir jetzt calls zum GL_ARRAY_BUFFER target machen dann wird der aktuelle gebindete buffer verwendet
    glBindBuffer(GL_ARRAY_BUFFER, VBO);


    float lineLength = 0.5f;
    std::vector<float> coordPoints;
    float color[3] = {0.7f, 0.7f, 0.7f};
    auto it = pointCloud.getVertices();
    for (auto i = 0; i < pointCloud.getVertexCount(); i++) {
        if (it->normal_x != -1 || it->normal_y != -1 || it->normal_z != -1) {
            coordPoints.push_back(it->x);
            coordPoints.push_back(it->y);
            coordPoints.push_back(it->z);
            coordPoints.push_back(color[0]);
            coordPoints.push_back(color[1]);
            coordPoints.push_back(color[2]);

            coordPoints.push_back(it->x + it->normal_x * lineLength);
            coordPoints.push_back(it->y + it->normal_y * lineLength);
            coordPoints.push_back(it->z + it->normal_z * lineLength);
            coordPoints.push_back(color[0]);
            coordPoints.push_back(color[1]);
            coordPoints.push_back(color[2]);
        }
        it++;
    }
    glBufferData(GL_ARRAY_BUFFER, coordPoints.size() * sizeof(float), coordPoints.data(), GL_STATIC_DRAW); //coordPoints.size() * sizeof(float), &coordPoints, GL_STATIC_DRAW);
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

void Window::dataStuffCoordSys(GLuint &VBO, GLuint &VAO) {

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