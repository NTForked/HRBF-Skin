#ifndef MAYAHRBFMAN_H
#define MAYAHRBFMAN_H

#include "MayaHRBF.h"
#include <vector>

/******************************************************************************
Implements a skeleton of HRBF nodes.
This is effectively a fake scenegraph of sorts.
******************************************************************************/

class MayaHRBFManager {
public:
	MayaHRBFManager(std::vector<int> jointHierarchy);
	~MayaHRBFManager();
	void recompute();

	// members
	std::vector<MayaHRBF*> m_HRBFs; // in the initial DF order
	int m_numJoints;
};

#endif