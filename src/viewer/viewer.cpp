//
//  viewer.cpp
//  Thanda

#include "viewer.hpp"

const char *attributeLocations[] = { "Position" }; // uhhh...

void printGLErrorLog()
{
	GLenum error = glGetError();
	if (error != GL_NO_ERROR) {
		std::cout << "CAUTION: OpenGL error " << error << ": ";
		const char *e =
			error == GL_INVALID_OPERATION ? "GL_INVALID_OPERATION" :
			error == GL_INVALID_ENUM ? "GL_INVALID_ENUM" :
			error == GL_INVALID_VALUE ? "GL_INVALID_VALUE" :
			error == GL_INVALID_INDEX ? "GL_INVALID_INDEX" :
			"unknown";
		std::cout << e << std::endl;

		// Throwing here allows us to use the debugger stack trace to track
		// down the error.
#ifndef __APPLE__
		// But don't do this on OS X. It might cause a premature crash.
		// http://lists.apple.com/archives/mac-opengl/2012/Jul/msg00038.html
		//throw;
#endif
	}
}

Viewer::Viewer() {

}

Viewer::~Viewer() {

}

void Viewer::loadShaders() {
	// load shader
	shaderProgram = glslUtility::createProgram(
		"../shaders/diff.vert.glsl",
		"../shaders/diff.frag.glsl", attributeLocations, 1);
	glUseProgram(shaderProgram);
	locationPos = glGetAttribLocation(shaderProgram, "vs_Position");
	locationNor = glGetAttribLocation(shaderProgram, "vs_Normal");
	locationCol = glGetAttribLocation(shaderProgram, "vs_Color");

	unifViewProj = glGetUniformLocation(shaderProgram, "u_projMatrix");
	unifModel = glGetUniformLocation(shaderProgram, "u_Model");
	unifModelInv = glGetUniformLocation(shaderProgram, "u_ModelInvTr");

	updateCamera();
}

int Viewer::init() {
	// initialize GLFW
	if (!glfwInit())
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); //We don't want the old OpenGL 

	// Open a window and create its OpenGL context
	GLFWwindow* window; // (In the accompanying source code, this variable is global)
	window = glfwCreateWindow(800, 600, "Viewer", NULL, NULL);
	if (window == NULL){
		fprintf(stderr, "Failed to open GLFW window.");
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(window); // Initialize GLEW

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

	glewExperimental = GL_TRUE; // Needed in core profile
	if (glewInit() != GLEW_OK) {
		return false;
	}
	printGLErrorLog();

	// load shaders
	loadShaders();

	// VAO
	glGenVertexArrays(1, &vao);

	// TODO: replace
	Geom *test_cube = new Geom();

	// window loop
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
		glfwWindowShouldClose(window) == 0) {

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0.5f, 0.7f, 1.0f, 1.0f);
		glEnable(GL_DEPTH_TEST);

		// handle drawing
		drawGeometry(test_cube);

		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}

void Viewer::drawGeometry(Geom *geometry) {
	glUseProgram(shaderProgram);
	glBindVertexArray(vao);

	int TRIANGLES = geometry->m_idx.size() / 3;

	// Activate our three kinds of vertex information
	glEnableVertexAttribArray(locationPos);
	glEnableVertexAttribArray(locationCol);
	glEnableVertexAttribArray(locationNor);
	glm::mat4 model = geometry->transformation;
	printGLErrorLog();

	// Set the 4x4 model transformation matrices
	glUniformMatrix4fv(unifModel, 1, GL_FALSE, &model[0][0]);
	// Also upload the inverse transpose for normal transformation
	const glm::mat4 modelInvTranspose = glm::inverse(glm::transpose(model));
	glUniformMatrix4fv(unifModelInv, 1, GL_FALSE, &modelInvTranspose[0][0]);
	printGLErrorLog();

	// Tell the GPU where the positions are: in the position buffer (4 components each)
	glBindBuffer(GL_ARRAY_BUFFER, geometry->m_vboPos);
	glVertexAttribPointer(locationPos, 3, GL_FLOAT, false, 0, NULL);
	printGLErrorLog();

	// Tell the GPU where the colors are: in the color buffer (4 components each)
	glBindBuffer(GL_ARRAY_BUFFER, geometry->m_vboCol);
	glVertexAttribPointer(locationCol, 3, GL_FLOAT, false, 0, NULL);
	printGLErrorLog();

	// Tell the GPU where the normals are: in the normal buffer (4 components each)
	glBindBuffer(GL_ARRAY_BUFFER, geometry->m_vboNor);
	glVertexAttribPointer(locationNor, 3, GL_FLOAT, false, 0, NULL);

	// Tell the GPU where the indices are: in the index buffer
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, geometry->m_vboIDX);

	// Draw the elements.
	glDrawElements(GL_TRIANGLES, TRIANGLES * 3, GL_UNSIGNED_INT, 0);

	// Shut off the information since we're done drawing.
	glDisableVertexAttribArray(locationPos);
	glDisableVertexAttribArray(locationCol);
	glDisableVertexAttribArray(locationNor);
}

void Viewer::updateCamera() {
	cameraPosition.x = zoom * sin(phi) * sin(theta);
	cameraPosition.z = zoom * cos(theta);
	cameraPosition.y = zoom * cos(phi) * sin(theta);
	cameraPosition += lookAt;

	int width = 800;
	int height = 600;
	projection = perspective(fovy, float(width) / float(height), zNear, zFar);
	mat4 view = glm::lookAt(cameraPosition, lookAt, vec3(0, 0, 1));
	projection = projection * view;

	glUseProgram(shaderProgram);
	glUniformMatrix4fv(unifViewProj, 1, GL_FALSE, &projection[0][0]);
}