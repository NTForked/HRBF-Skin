#include "MayaHRBF.h"

MayaHRBF::MayaHRBF(std::string &name, MMatrix &invBindTF) {
	m_parent = NULL;
	m_name = name;
	m_invBindTF = invBindTF;
	m_bindTF = invBindTF.inverse();
	m_bindPosition.x = m_bindTF(3, 0);
	m_bindPosition.y = m_bindTF(3, 1);
	m_bindPosition.z = m_bindTF(3, 2);

	return;
}

MayaHRBF::~MayaHRBF() {

}