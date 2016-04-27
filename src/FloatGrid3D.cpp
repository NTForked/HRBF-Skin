#include "FloatGrid3D.hpp"

FloatGrid3D::FloatGrid3D(int resX, int resY, int resZ,
	float minX, float minY, float minZ,
	float maxX, float maxY, float maxZ) {

	m_res = glm::ivec3(resX, resY, resZ);
	m_min = glm::vec3(minX, minY, minZ);
	m_max = glm::vec3(maxX, maxY, maxZ);
	m_numCells = resX * resY * resZ;
	m_cells.resize(m_numCells);
	m_cellWidth.x = (maxX - minX) / (float)resX;
	m_cellWidth.y = (maxY - minY) / (float)resY;
	m_cellWidth.z = (maxZ - minZ) / (float)resZ;

}

FloatGrid3D::~FloatGrid3D() {

	m_cells.clear();
	return;
}

int FloatGrid3D::threeDto1D(int x, int y, int z) {

	// out of bounds? return something indicative.
	if (x < 0 || y < 0 || z < 0 ||
		x >= m_res.x || y >= m_res.y || z > m_res.z) {
		return -1;
	}
	// http://www.cplusplus.com/forum/general/137677/
	return x * m_res.y * m_res.z + y * m_res.z + z;
}

bool FloatGrid3D::checkBounds(float x, float y, float z) {

	return (
		x >= m_min.x && x <= m_max.x &&
		y >= m_min.y && y <= m_max.y &&
		z >= m_min.z && z <= m_max.z
		);
}

void FloatGrid3D::coordToIDX(float x, float y, float z, 
	int &ix, int &iy, int &iz) {

	// compute the IDX that is to the bottom left of the given coordinate
	ix = (x - m_min.x) / (float)m_cellWidth.x;
	iy = (y - m_min.y) / (float)m_cellWidth.y;
	iz = (z - m_min.z) / (float)m_cellWidth.z;
}

void FloatGrid3D::idxToCoord(int ix, int iy, int iz,
	float &x, float &y, float &z
	) {

	x = ix * m_cellWidth.x + m_min.x;
	y = iy * m_cellWidth.y + m_min.y;
	z = iz * m_cellWidth.z + m_min.z;
}

void FloatGrid3D::clear(float val) {

	for (int i = 0; i < m_numCells; i++) {
		m_cells[i] = val;
	}
}

float FloatGrid3D::getCell(int x, int y, int z) {

	int idx = threeDto1D(x, y, z);
	if (idx >= 0 && idx < m_numCells) {
		return m_cells[idx];
	}
	return -HUGE_VAL;
}

float FloatGrid3D::getByCoordinate(float x, float y, float z) {

	int ix, iy, iz;
	coordToIDX(x, y, z, ix, iy, iz);
	return getCell(ix, iy, iz);
}

bool FloatGrid3D::setCell(int x, int y, int z, float val) {

	int idx = threeDto1D(x, y, z);
	if (idx < 0 || idx >= m_numCells) {
		return false;
	}
	m_cells[idx] = val;
	return true;
}

bool FloatGrid3D::setByCoordinate(float x, float y, float z, float val) {

	if (!checkBounds(x, y, z)) {
		return false;
	}
	int ix, iy, iz;
	coordToIDX(x, y, z, ix, iy, iz);
	int idx = threeDto1D(ix, iy, iz);
	m_cells[idx] = val;
	return true;
}

float lerp(float t, float valA, float valB) {

	return (1.0f - t)*valA + t*valB; // more precise, according to wikipedia
}

bool FloatGrid3D::trilinear(float x, float y, float z, float &ret) {

	if (!checkBounds(x, y, z)) {
		return false;
	}
	/****************************************************
	   D --- C      y
      /|    /|      |    
	 / H   / G      o--x    
	A --- B /      /                
	|     |/      z       
	E --- F      

	We'll consider H to be the min, what coordToIDX returns.
	Trilinear interpolation:
	-lerp by z
		-lerp A-D = AD
		-lerp E-H = EH
		-lerp B-C = BC
		-lerp F-G = FG
	-lerp by y
		-lerp FG-BC = FGBC
		-lerp AD-EH = ADEH
	-lerp by x
		-lerp FGBC-ADEH

	H = minX, minY, minZ
	G = maxX, minY, minZ
	E = minX, minY, maxZ
	F = maxX, minY, maxX

	D = minX, maxY, minZ
	C = maxX, maxY, minZ
	A = minX, maxY, maxZ
	B = maxX, maxY, maxX
	*****************************************************/
	int minX, minY, minZ;
	int maxX, maxY, maxZ;
	float fminX, fminY, fminZ;
	float fmaxX, fmaxY, fmaxZ;

	coordToIDX(x, y, z, minX, minY, minZ);
	maxX = minX + 1;
	maxY = minY + 1;
	maxZ = minZ + 1;

	idxToCoord(minX, minY, minZ, fminX, fminY, fminZ);
	idxToCoord(maxX, maxY, maxZ, fmaxX, fmaxY, fmaxZ);

	float H = getCell(minX, minY, minZ);
	float G = getCell(maxX, minY, minZ);
	float E = getCell(minX, minY, maxZ);
	float F = getCell(maxX, minY, maxX);

	float D = getCell(minX, maxY, minZ);
	float C = getCell(maxX, maxY, minZ);
	float A = getCell(minX, maxY, maxZ);
	float B = getCell(maxX, maxY, maxX);

	// we'll do some reuse here to avoid adding stuff on the stack

	// lerp by z
	float t = (z - fminZ) / (fmaxZ - fminZ);
	A = lerp(t, D, A);
	E = lerp(t, H, E);
	B = lerp(t, C, B);
	F = lerp(t, G, F);

	// lerp by y
	t = (y - fminY) / (fmaxY - fminY);
	F = lerp(t, F, B);
	A = lerp(t, E, A);

	t = (x - fminX) / (fmaxX - fminX);
	ret = lerp(t, A, F);
	return true;
}

void FloatGrid3D::exportToFile(std::string filename) {
	std::ofstream file;
	file.open(filename);
	float fx, fy, fz;
	for (int x = 0; x < m_res.x; x++) {
		for (int y = 0; y < m_res.y; y++) {
			for (int z = 0; z < m_res.z; z++) {
				idxToCoord(x, y, z, fx, fy, fz);
				file << fx << " " << fy << " " << fz << " " << getCell(x, y, z) << "\n";
			}
		}
	}
	file.close();
}