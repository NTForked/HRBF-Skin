#include "MayaHRBFManager.h"

MayaHRBFManager::MayaHRBFManager() {

}

MayaHRBFManager::~MayaHRBFManager() {
	// wipe whatever used to be in the hierarchy
	for (int i = 0; i < m_numJoints; i++) {
		if (m_HRBFs.at(i) != NULL)
			delete m_HRBFs.at(i);
	}
	m_HRBFs.clear();
}

void MayaHRBFManager::buldHRBFs(std::vector<int> jointHierarchy, std::vector<std::string> names,
	MMatrixArray &binds,
	MArrayDataHandle& weightListHandle, MItGeometry& iter, MObject &weights) {
	// wipe whatever used to be in the hierarchy
	for (int i = 0; i < m_numJoints; i++) {
		if (m_HRBFs.at(i) != NULL)
			delete m_HRBFs.at(i);
	}
	m_HRBFs.clear();

	m_numJoints = jointHierarchy.size();
	// go ahead and make a bunch of empty HRBFs
	// in the same order that matrices are expected to be provided
	for (int i = 0; i < m_numJoints; i++) {
		MayaHRBF *hrbf = new MayaHRBF(names.at(i), binds[i]);
		m_HRBFs.push_back(hrbf);
	}

	// populate hierarchy information
	int parentIDX;
	for (int i = 0; i < m_numJoints; i++) {
		parentIDX = jointHierarchy[i];
		if (parentIDX >= 0) {
			m_HRBFs[i]->m_parent = m_HRBFs[parentIDX];
			m_HRBFs[parentIDX]->m_children.push_back(m_HRBFs[i]);
		}
	}

	// set up bony points for all the HRBFs. needed for ctl point culling, so do this before adding points.
	for (int i = 0; i < m_numJoints; i++) {
		m_HRBFs[i]->setupBones();
	}

	/***** sample points for the HRBFs *****/
	// sample the geometry and deposit the appropriate normals, etc. into each HRBF.
	// Iterate through each point in the geometry.
	for (; !iter.isDone(); iter.next()) {
		MPoint pt = iter.position();
		MVector nor = iter.normal();

		// get the weights for this point
		MArrayDataHandle weightsHandle = weightListHandle.inputValue().child(weights);
		// compute the skinning -> TODO: what's the order that the weights are given in? Appears to just be maya list relatives order.
		for (int i = 0; i< m_numJoints; ++i) {
			if (MS::kSuccess == weightsHandle.jumpToElement(i)) {
				if (weightsHandle.inputValue().asDouble() > WEIGHT_CUTOFF)
					m_HRBFs[i]->addVertex(pt, nor);
			}
		}

		// advance the weight list handle
		weightListHandle.next();
	}

	/***** set up each individual HRBF *****/
	for (int i = 0; i < m_numJoints; i++) {
		m_HRBFs[i]->compute();
	}

}

void MayaHRBFManager::debugOutputToFile(std::string nodeName) {
	for (int i = 0; i < m_numJoints; i++) {
		if (m_HRBFs[i]->m_name == nodeName) {
			m_HRBFs[i]->printHRBFSamplingDebug();
			break;
		}
	}
}