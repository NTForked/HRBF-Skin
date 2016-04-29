#ifndef FLOATGRID3D_H
#define FLOATGRID3D_H

#include <glm\glm.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <maya/MVector.h>
#include <maya/MPoint.h>
#include <maya/MMatrix.h>
#include <algorithm>

class FloatGrid3D {
public:
	FloatGrid3D(int resX, int resY, int resZ,
		float minX, float minY, float minZ,
		float maxX, float maxY, float maxZ);
	~FloatGrid3D();
	void resizeAABB(float minX, float minY, float minZ,
		float maxX, float maxY, float maxZ);
	void resizeAABB(MPoint min, MPoint max);


	void getWorldAABB(MMatrix tf, MPoint &wMin, MPoint &wMax);

	// members
	std::vector<float> m_cells;
	int m_numCells;
	glm::ivec3 m_res;
	glm::vec3 m_min;
	glm::vec3 m_max;
	glm::vec3 m_cellWidth;

	/********************* helpers **********************/

	// return a 1D index given a 3D index
	int threeDto1D(int x, int y, int z); 
	// index of grid coordinate to the bottom/left/back
	bool checkBounds(float x, float y, float z);
	void coordToIDX(float x, float y, float z, int &ix, int &iy, int &iz);
	float distToIDX(float x, float y, float z, int ix, int iy, int iz);
	void nearestIDX(float x, float y, float z, int &ix, int &iy, int &iz);
	void idxToCoord(int ix, int iy, int iz, float &x, float &y, float &z);
	MPoint idxToCoord(int ix, int iy, int iz);

	MVector idxToMVector(int ix, int iy, int iz);

	/***************** accessor/setters *****************/
	void clear(float val); // set all cells to val
	float getCell(int x, int y, int z); // access the cell at x, y, z
	float getByCoordinate(float x, float y, float z); // access cell by coordinate
	bool setCell(int x, int y, int z, float val); // access the cell at x, y, z
	bool setByCoordinate(float x, float y, float z, float val); // access cell by coordinate
	bool trilinear(float x, float y, float z, float &ret); // trilinear interpolation

	// maya doesn't seem to let plugins export files, so we'll just have to view it in the debugger
	void exportToDebugString(std::string nodeName);
};

#endif