//
//  viewer.hpp
//  Thanda

#ifndef viewer_hpp
#define viewer_hpp

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
#include "../geom/geom.hpp"
#include "glslUtility.hpp"

using namespace glm;
using namespace std;

class Viewer {
public:
	// member variables for camera parameters
	const float fovy = (float)(M_PI / 4);
	const float zNear = 0.10f;
	const float zFar = 200.0f;
	mat4 projection;
	float theta = 1.22f;
	float phi = -0.65f;
	float zoom = 150.0f;
	vec3 lookAt = vec3(0.0f, 0.0f, 0.0f);
	vec3 cameraPosition;

	GLuint shaderProgram; // shader program handle
	GLuint locationPos; // position array handle
	GLuint locationNor; // normal array handle
	GLuint locationCol; // color array handle
	GLuint unifViewProj; // camera matrix handle
	GLuint unifModel; // model matrix handle
	GLuint unifModelInv; // model matrix inverse handle
	GLuint vao;

	Viewer();
	~Viewer();
	void loadShaders();
	int init();
	void drawGeometry(Geom *geometry);
	void updateCamera();
};

#endif /* viewer_hpp */
