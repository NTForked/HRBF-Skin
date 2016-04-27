#ifndef MAYAHRBFMAN_H
#define MAYAHRBFMAN_H

#include "MayaHRBF.h"
#include <vector>
#include "FloatGrid3D.hpp"

#include <maya/MMatrixArray.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MDataHandle.h>
#include <maya/MItGeometry.h>

#define HRBF_COMPRES 128
#define WEIGHT_CUTOFF 0.4

/******************************************************************************
Implements a skeleton of HRBF nodes.
This is effectively a fake scenegraph of sorts.
******************************************************************************/

class MayaHRBFManager {
public:
	MayaHRBFManager();
	~MayaHRBFManager();
	void buldHRBFs(std::vector<int> jointHierarchy, std::vector<std::string> names,
		MMatrixArray &binds,
		MArrayDataHandle& weightListHandle, MItGeometry& iter, MObject &weights);
	// we need weights to access things in the weightListHandle

	// members
	std::vector<MayaHRBF*> m_HRBFs; // parallel to MMatrixArrays transforms and binds
	std::vector<float> m_isoVals; // parallel to MItGeometry iter
	int m_numJoints;

	void compose(); // TODO: implement

	void correct(); // TODO: fill out and implement

	void debugOutputToFile(std::string nodeName);
};

#endif