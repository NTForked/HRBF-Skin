#ifndef MAYAHRBF_H
#define MAYAHRBF_H

#include <maya/MVector.h>
#include <maya/MMatrix.h>
#include "FloatGrid3D.hpp"
#include <vector>

class MayaHRBF {
public:
	MayaHRBF(std::string &name);
	~MayaHRBF();

	// members
	std::string m_name;
	MayaHRBF* m_parent;
	std::vector<MayaHRBF*> m_children;
};

#endif