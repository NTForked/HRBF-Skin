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

	void compose(MMatrixArray&  transforms, int numTransforms); // TODO: implement

	void correct(MItGeometry& iter); // TODO: implement

	void debugSamplesToConsole(std::string nodeName);
	void debugValuesToConsole(std::string nodeName);
	void debugCompositionToConsole();

};

#endif