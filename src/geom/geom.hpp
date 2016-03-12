//
//  geom.hpp
//  Defaults to a cube with color normals

#ifndef geom_hpp
#define geom_hpp

#include <glm/glm.hpp>
#include "glslUtil\glslUtility.hpp"
#include "vector"

using namespace glm;
using namespace std;

class Geom {
public:
	GLuint m_vboPos; // pointer to vertices on GPU
	GLuint m_vboCol; // pointer to colors on GPU
	GLuint m_vboNor; // pointer to normals on GPU
	GLuint m_vboIDX; // pointer to indices on GPU

	// CPU side
	vector<vec3> m_pos;
	vector<vec3> m_nor;
	vector<vec3> m_col;
	vector<unsigned int> m_idx;
	mat4 transformation;

	Geom();
	~Geom();

	void updateOnGPU();
};

#endif /* geom_hpp */
