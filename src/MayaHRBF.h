#ifndef MAYAHRBF_H
#define MAYAHRBF_H

#include <maya/MVector.h>
#include <maya/MMatrix.h>
#include <vector>

class MayaHRBF {
public:
	MayaHRBF();
	~MayaHRBF();

	// members
	MayaHRBF* m_parent;
	std::vector<MayaHRBF*> m_children;
};

#endif