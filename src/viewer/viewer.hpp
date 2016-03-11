//
//  viewer.hpp
//  Thanda

#ifndef viewer_hpp
#define viewer_hpp

#include <stdio.h>
#include <stdlib.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

using namespace glm;
using namespace std;

class Viewer {
public:
	Viewer();
	~Viewer();
	int init();
};

#endif /* viewer_hpp */
