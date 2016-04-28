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
	std::vector<int> hrbfCandidates;
	std::vector<double> candidateWeights;
	MPoint pt;
	MVector nor;
	int numCandidates;
	double bestWeight;
	int bestHRBF;

	for (; !iter.isDone(); iter.next()) {
		pt = iter.position();
		nor = iter.normal();
		hrbfCandidates.clear();
		candidateWeights.clear();
		numCandidates = 0;

		// get the weights for this point
		MArrayDataHandle weightsHandle = weightListHandle.inputValue().child(weights);
		// compute the skinning -> TODO: what's the order that the weights are given in? Appears to just be maya list relatives order.
		for (int i = 0; i< m_numJoints; ++i) {
			// give this vertex to the HRBF that is weighted most heavily towards

			if (MS::kSuccess == weightsHandle.jumpToElement(i)) {
				candidateWeights.push_back(weightsHandle.inputValue().asDouble());
				hrbfCandidates.push_back(i);
				numCandidates++;
			}
		}

		// figure out which HRBF to add this vertex to
		bestWeight = -HUGE_VAL;
		bestHRBF = -1;
		for (int i = 0; i < numCandidates; i++) {
			if (candidateWeights[i] > bestWeight) {
				bestWeight = candidateWeights[i];
				bestHRBF = hrbfCandidates[i];
			}
		}
		m_HRBFs[bestHRBF]->addVertex(pt, nor);

		// advance the weight list handle
		weightListHandle.next();
	}

	/***** set up each individual HRBF *****/
	for (int i = 0; i < m_numJoints; i++) {
		std::cout << "setting up " << names[i] << "...";
		m_HRBFs[i]->compute();
		std::cout << "done!" << std::endl;
	}

}

void MayaHRBFManager::compose(MMatrixArray &transforms, int numTransforms) {

}

void MayaHRBFManager::correct(MItGeometry& iter) {

}

void MayaHRBFManager::debugSamplesToConsole(std::string nodeName) {
	for (int i = 0; i < m_numJoints; i++) {
		if (m_HRBFs[i]->m_name == nodeName) {
			m_HRBFs[i]->printHRBFSamplingDebug();
			break;
		}
	}
}

void MayaHRBFManager::debugValuesToConsole(std::string nodeName) {
	for (int i = 0; i < m_numJoints; i++) {
		if (m_HRBFs[i]->m_name == nodeName) {
			m_HRBFs[i]->printHRBF();
			break;
		}
	}
}

void MayaHRBFManager::debugCompositionToConsole(MMatrixArray &transforms, int numTransforms) {
	compose(transforms, numTransforms);
}