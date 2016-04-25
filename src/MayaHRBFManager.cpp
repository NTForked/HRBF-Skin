#include "MayaHRBFManager.h"

MayaHRBFManager::MayaHRBFManager(std::vector<int> jointHierarchy) {
	m_numJoints = jointHierarchy.size();
	// go ahead and make a bunch of empty HRBFs
	// in the same order that matrices are expected to be provided
	for (int i = 0; i < m_numJoints; i++) {
		MayaHRBF *hrbf = new MayaHRBF();
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
}

MayaHRBFManager::~MayaHRBFManager() {

}