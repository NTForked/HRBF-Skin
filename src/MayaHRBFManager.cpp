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
		if (parentIDX > 0) {
			m_HRBFs[i]->m_parent = m_HRBFs[parentIDX];
			m_HRBFs[parentIDX]->m_children.push_back(m_HRBFs[i]);
		}
	}

	// set up the actual HRBFs
}
