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

void FloatGrid3D::resizeAABB(float minX, float minY, float minZ,
	float maxX, float maxY, float maxZ) {

	m_min = glm::vec3(minX, minY, minZ);
	m_max = glm::vec3(maxX, maxY, maxZ);
	m_cellWidth.x = (maxX - minX) / (float)m_res.x;
	m_cellWidth.y = (maxY - minY) / (float)m_res.y;
	m_cellWidth.z = (maxZ - minZ) / (float)m_res.z;
}

void FloatGrid3D::resizeAABB(MPoint min, MPoint max) {
	m_min = glm::vec3(min.x, min.y, min.z);
	m_max = glm::vec3(max.x, max.y, max.z);
	m_cellWidth.x = (max.x - min.x) / (float)m_res.x;
	m_cellWidth.y = (max.y - min.y) / (float)m_res.y;
	m_cellWidth.z = (max.z - min.z) / (float)m_res.z;
}


void FloatGrid3D::getWorldAABB(MMatrix tf, MPoint &wMin, MPoint &wMax) {
	// transform each of the 8 points into world space
	// walk over them and compute the world AABB
	wMin.x = HUGE_VAL;
	wMin.y = HUGE_VAL;
	wMin.z = HUGE_VAL;

	wMax.x = -HUGE_VAL;
	wMax.y = -HUGE_VAL;
	wMax.z = -HUGE_VAL;

	MPoint corner;

	corner = MPoint(m_min.x, m_min.y, m_min.z, 1.0) * tf;
	wMin.x = std::min(corner.x, wMin.x);
	wMin.y = std::min(corner.y, wMin.y);
	wMin.z = std::min(corner.z, wMin.z);
	wMax.x = std::max(corner.x, wMax.x);
	wMax.y = std::max(corner.y, wMax.y);
	wMax.z = std::max(corner.z, wMax.z);

	corner = MPoint(m_max.x, m_min.y, m_min.z, 1.0) * tf;
	wMin.x = std::min(corner.x, wMin.x);
	wMin.y = std::min(corner.y, wMin.y);
	wMin.z = std::min(corner.z, wMin.z);
	wMax.x = std::max(corner.x, wMax.x);
	wMax.y = std::max(corner.y, wMax.y);
	wMax.z = std::max(corner.z, wMax.z);

	corner = MPoint(m_max.x, m_max.y, m_min.z, 1.0) * tf;
	wMin.x = std::min(corner.x, wMin.x);
	wMin.y = std::min(corner.y, wMin.y);
	wMin.z = std::min(corner.z, wMin.z);
	wMax.x = std::max(corner.x, wMax.x);
	wMax.y = std::max(corner.y, wMax.y);
	wMax.z = std::max(corner.z, wMax.z);

	corner = MPoint(m_min.x, m_max.y, m_min.z, 1.0) * tf;
	wMin.x = std::min(corner.x, wMin.x);
	wMin.y = std::min(corner.y, wMin.y);
	wMin.z = std::min(corner.z, wMin.z);
	wMax.x = std::max(corner.x, wMax.x);
	wMax.y = std::max(corner.y, wMax.y);
	wMax.z = std::max(corner.z, wMax.z);

	corner = MPoint(m_min.x, m_min.y, m_max.z, 1.0) * tf;
	wMin.x = std::min(corner.x, wMin.x);
	wMin.y = std::min(corner.y, wMin.y);
	wMin.z = std::min(corner.z, wMin.z);
	wMax.x = std::max(corner.x, wMax.x);
	wMax.y = std::max(corner.y, wMax.y);
	wMax.z = std::max(corner.z, wMax.z);

	corner = MPoint(m_max.x, m_min.y, m_max.z, 1.0) * tf;
	wMin.x = std::min(corner.x, wMin.x);
	wMin.y = std::min(corner.y, wMin.y);
	wMin.z = std::min(corner.z, wMin.z);
	wMax.x = std::max(corner.x, wMax.x);
	wMax.y = std::max(corner.y, wMax.y);
	wMax.z = std::max(corner.z, wMax.z);

	corner = MPoint(m_max.x, m_max.y, m_max.z, 1.0) * tf;
	wMin.x = std::min(corner.x, wMin.x);
	wMin.y = std::min(corner.y, wMin.y);
	wMin.z = std::min(corner.z, wMin.z);
	wMax.x = std::max(corner.x, wMax.x);
	wMax.y = std::max(corner.y, wMax.y);
	wMax.z = std::max(corner.z, wMax.z);

	corner = MPoint(m_min.x, m_max.y, m_max.z, 1.0) * tf;
	wMin.x = std::min(corner.x, wMin.x);
	wMin.y = std::min(corner.y, wMin.y);
	wMin.z = std::min(corner.z, wMin.z);
	wMax.x = std::max(corner.x, wMax.x);
	wMax.y = std::max(corner.y, wMax.y);
	wMax.z = std::max(corner.z, wMax.z);
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

float FloatGrid3D::distToIDX(float x, float y, float z, int ix, int iy, int iz)
{
	glm::vec3 pt(x, y, z);
	float fx, fy, fz;
	idxToCoord(ix, iy, iz, fx, fy, fz);
	glm::vec3 pt2(fx, fy, fz);
	return (pt - pt2).length();
}

void FloatGrid3D::nearestIDX(float x, float y, float z,
	int &ix, int &iy, int &iz) {

	coordToIDX(x, y, z, ix, iy, iz);
	float bestDist = distToIDX(x, y, z, ix, iy, iz);

	// check other 7 nearest points to see if one is closer
	float candDist;
	int bestX = ix;
	int bestY = iy;
	int bestZ = iz;
	candDist = distToIDX(x, y, z, ix, iy, iz + 1);
	if (candDist < bestDist) {
		bestDist = candDist;
		bestX = ix;
		bestY = iy;
		bestZ = iz + 1;
	}
	candDist = distToIDX(x, y, z, ix, iy + 1, iz);
	if (candDist < bestDist) {
		bestDist = candDist;
		bestX = ix;
		bestY = iy + 1;
		bestZ = iz;
	}
	candDist = distToIDX(x, y, z, ix + 1, iy, iz);
	if (candDist < bestDist) {
		bestDist = candDist;
		bestX = ix + 1;
		bestY = iy;
		bestZ = iz;
	}
	candDist = distToIDX(x, y, z, ix + 1, iy + 1, iz);
	if (candDist < bestDist) {
		bestDist = candDist;
		bestX = ix + 1;
		bestY = iy + 1;
		bestZ = iz;
	}
	candDist = distToIDX(x, y, z, ix, iy + 1, iz + 1);
	if (candDist < bestDist) {
		bestDist = candDist;
		bestX = ix;
		bestY = iy + 1;
		bestZ = iz + 1;
	}
	candDist = distToIDX(x, y, z, ix + 1, iy, iz + 1);
	if (candDist < bestDist) {
		bestDist = candDist;
		bestX = ix + 1;
		bestY = iy;
		bestZ = iz + 1;
	}
	candDist = distToIDX(x, y, z, ix + 1, iy + 1, iz + 1);
	if (candDist < bestDist) {
		bestDist = candDist;
		bestX = ix + 1;
		bestY = iy + 1;
		bestZ = iz + 1;
	}
	ix = bestX;
	iy = bestY;
	iz = bestZ;
	return;
}

MPoint FloatGrid3D::idxToCoord(int ix, int iy, int iz) {

	float x = ix * m_cellWidth.x + m_min.x;
	float y = iy * m_cellWidth.y + m_min.y;
	float z = iz * m_cellWidth.z + m_min.z;
	return MPoint(x, y, z, 1.0);
}

void FloatGrid3D::idxToCoord(int ix, int iy, int iz,
	float &x, float &y, float &z
	) {

	x = ix * m_cellWidth.x + m_min.x;
	y = iy * m_cellWidth.y + m_min.y;
	z = iz * m_cellWidth.z + m_min.z;
}

MVector FloatGrid3D::idxToMVector(int ix, int iy, int iz) {
	
	MVector ret;
	ret.x = ix * m_cellWidth.x + m_min.x;
	ret.y = iy * m_cellWidth.y + m_min.y;
	ret.z = iz * m_cellWidth.z + m_min.z;
	return ret;
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

void FloatGrid3D::exportToDebugString(std::string nodeName) {
	std::cout << nodeName.c_str() << "\n";
	float fx, fy, fz;
	float val;
	for (int x = 0; x < m_res.x; x += 3) {
		for (int y = 0; y < m_res.y; y += 3) {
			for (int z = 0; z < m_res.z; z += 3) {
				idxToCoord(x, y, z, fx, fy, fz);
				val = getCell(x, y, z);
				if (val > 0.0001f)
					std::cout << fx << " " << fy << " " << fz << " " << val << "\n";
			}
		}
	}
	// print bounding box values
	std::cout << m_min.x << " " << m_min.y << " " << m_min.z << " " << 1000.0f << "\n";
	std::cout << m_max.x << " " << m_max.y << " " << m_max.z << " " << 1000.0f << "\n";

	return;
}