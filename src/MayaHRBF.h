#ifndef MAYAHRBF_H
#define MAYAHRBF_H

#include <maya/MVector.h>
#include <maya/MMatrix.h>
#include "FloatGrid3D.hpp"
#include <vector>
#include <algorithm>

#define HRBF_RES 32
#define CULL_H 0.05f

class MayaHRBF {
public:
	MayaHRBF(std::string &name, MMatrix &invBindTF);
	~MayaHRBF();

	void setupBones();
	void addVertex(MPoint pos, MVector nor); // take vertex into local space. handle culling.
	void compute();

	void printHRBF(); // TODO: implement!

	// members
	std::string m_name;
	MayaHRBF* m_parent;
	std::vector<MayaHRBF*> m_children;
	std::vector<MPoint> m_positions;
	std::vector<MVector> m_normals;

	std::vector<MPoint> m_endPs;
	std::vector<MVector> m_endNs;
	MVector m_boneLength;
	double m_boneLengthSquared;
	MPoint m_bindPositionLocal;

	MMatrix m_invBindTF;
	MMatrix m_bindTF;
	MPoint m_bindPosition; // world space

	FloatGrid3D *f_vals;
	FloatGrid3D *f_gradX;
	FloatGrid3D *f_gradY;
	FloatGrid3D *f_gradZ;
	FloatGrid3D *f_gradMag;
};

#endif