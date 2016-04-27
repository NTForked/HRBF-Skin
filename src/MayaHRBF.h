#ifndef MAYAHRBF_H
#define MAYAHRBF_H

#include <maya/MVector.h>
#include <maya/MMatrix.h>
#include "FloatGrid3D.hpp"
#include <vector>

class MayaHRBF {
public:
	MayaHRBF(std::string &name, MMatrix &invBindTF);
	~MayaHRBF();

	// members
	std::string m_name;
	MayaHRBF* m_parent;
	std::vector<MayaHRBF*> m_children;
	std::vector<MPoint> m_positions;
	std::vector<MVector> m_normals;
	MMatrix m_invBindTF;
	MMatrix m_bindTF;
	MPoint m_bindPosition;
};

#endif