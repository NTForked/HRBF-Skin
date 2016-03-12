//
//  geom.cpp
//  Thanda

#include "geom.hpp"

Geom::Geom() {
	glGenBuffers(1, &m_vboPos);
	glGenBuffers(1, &m_vboCol);
	glGenBuffers(1, &m_vboNor);
	glGenBuffers(1, &m_vboIDX);

	// generate vertices and indices for cube

	m_pos.clear();
	m_nor.clear();
	m_col.clear();
	m_idx.clear();

	float halfWidth = 0.5f;

	// vertices
	vec3 v0 = vec3(halfWidth, halfWidth, halfWidth);
	vec3 v1 = vec3(-halfWidth, halfWidth, halfWidth);
	vec3 v2 = vec3(-halfWidth, -halfWidth, halfWidth);
	vec3 v3 = vec3(halfWidth, -halfWidth, halfWidth);
	vec3 v4 = vec3(halfWidth, -halfWidth, -halfWidth);
	vec3 v5 = vec3(halfWidth, halfWidth, -halfWidth);
	vec3 v6 = vec3(-halfWidth, halfWidth, -halfWidth);
	vec3 v7 = vec3(-halfWidth, -halfWidth, -halfWidth);

	m_pos.push_back(v0); m_pos.push_back(v1); m_pos.push_back(v2); m_pos.push_back(v3);
	m_pos.push_back(v0); m_pos.push_back(v3); m_pos.push_back(v4); m_pos.push_back(v5);
	m_pos.push_back(v0); m_pos.push_back(v5); m_pos.push_back(v6); m_pos.push_back(v1);
	m_pos.push_back(v1); m_pos.push_back(v6); m_pos.push_back(v7); m_pos.push_back(v2);
	m_pos.push_back(v7); m_pos.push_back(v4); m_pos.push_back(v3); m_pos.push_back(v2);
	m_pos.push_back(v4); m_pos.push_back(v7); m_pos.push_back(v6); m_pos.push_back(v5);

	// normals

	vec3 n0 = vec3(0, 0, 1);
	vec3 n1 = vec3(1, 0, 0);
	vec3 n2 = vec3(0, 1, 0);
	vec3 n3 = vec3(-1, 0, 0);
	vec3 n4 = vec3(0, -1, 0);
	vec3 n5 = vec3(0, 0, -1);

	m_nor.push_back(n0); m_nor.push_back(n0); m_nor.push_back(n0); m_nor.push_back(n0);
	m_nor.push_back(n1); m_nor.push_back(n1); m_nor.push_back(n1); m_nor.push_back(n1);
	m_nor.push_back(n2); m_nor.push_back(n2); m_nor.push_back(n2); m_nor.push_back(n2);
	m_nor.push_back(n3); m_nor.push_back(n3); m_nor.push_back(n3); m_nor.push_back(n3);
	m_nor.push_back(n4); m_nor.push_back(n4); m_nor.push_back(n4); m_nor.push_back(n4);
	m_nor.push_back(n5); m_nor.push_back(n5); m_nor.push_back(n5); m_nor.push_back(n5);

	// indices
	int arr[] = {
		0, 1, 2, 0, 2, 3,       // Front
		4, 5, 6, 4, 6, 7,       // Right
		8, 9, 10, 8, 10, 11,    // Top
		12, 13, 14, 12, 14, 15, // Left
		16, 17, 18, 16, 18, 19, // Bottom
		20, 21, 22, 20, 22, 23, // Back
	};

	for (int i = 0; i < 36; i++)
	{
		m_idx.push_back(arr[i]);
	}

	//// indices
	//indices_ = {
	//    0, 1, 2, 0, 2, 3,       // Front
	//    4, 5, 6, 4, 6, 7,       // Right
	//    8, 9, 10, 8, 10, 11,    // Top
	//    12, 13, 14, 12, 14, 15, // Left
	//    16, 17, 18, 16, 18, 19, // Bottom
	//    20, 21, 22, 20, 22, 23, // Back
	//};

	for (int i = 0; i < m_nor.size(); i++)
	{
		m_col.push_back(m_nor.at(i));
	}

	updateOnGPU();
}

Geom::~Geom() {
	glDeleteBuffers(1, &m_vboPos);
	glDeleteBuffers(1, &m_vboCol);
	glDeleteBuffers(1, &m_vboNor);
	glDeleteBuffers(1, &m_vboIDX);
}

void Geom::updateOnGPU() {
	int numV = m_pos.size();
	int numTris = m_idx.size() / 3;

	GLsizei SIZE_ATR = sizeof(glm::vec3);
	GLsizei allTris = numTris * sizeof(GLuint) * 3;

	// put data into vertex buffer objects
	// bind and buffer
	glBindBuffer(GL_ARRAY_BUFFER, m_vboPos);
	glBufferData(GL_ARRAY_BUFFER, numV * SIZE_ATR, &m_pos[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, m_vboNor);
	glBufferData(GL_ARRAY_BUFFER, numV * SIZE_ATR, &m_nor[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, m_vboCol);
	glBufferData(GL_ARRAY_BUFFER, numV * SIZE_ATR, &m_col[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vboIDX);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, allTris, &m_idx[0], GL_STATIC_DRAW);
}