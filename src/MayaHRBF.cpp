#include "MayaHRBF.h"

MayaHRBF::MayaHRBF(std::string &name, MMatrix &invBindTF) {
	m_parent = NULL;
	m_name = name;
	m_invBindTF = invBindTF;
	m_bindTF = invBindTF.inverse();
	m_bindPosition.x = m_bindTF(3, 0);
	m_bindPosition.y = m_bindTF(3, 1);
	m_bindPosition.z = m_bindTF(3, 2);
	m_bindPositionLocal = invBindTF * m_bindPosition;

	f_vals = NULL;
	f_gradX = NULL;
	f_gradY = NULL;
	f_gradZ = NULL;
	f_gradMag = NULL;
	return;
}

MayaHRBF::~MayaHRBF() {
	if (f_vals != NULL) delete f_vals;
	if (f_gradX != NULL) delete f_gradX;
	if (f_gradY != NULL) delete f_gradY;
	if (f_gradZ != NULL) delete f_gradZ;
	if (f_gradMag != NULL) delete f_gradMag;

}

void MayaHRBF::setupBones() {
	/***** get "bone end points" and normals *****/
	// look at each of the children's points
	int numChildren = m_children.size();
	MPoint endP;
	MVector endN;
	MPoint endPavg;
	for (int i = 0; i < numChildren; i++) {
		endP = m_children.at(i)->m_bindPosition;
		endN = (endP - m_bindPosition);
		endN.normalize();
		m_endPs.push_back(m_invBindTF * endP);
		m_endNs.push_back(m_invBindTF * endN);
		endPavg += endP;
	}
	// average the points to figure out what normal to use here.
	endP = m_bindPosition;
	endN = (endPavg / (float)numChildren) - m_bindPosition;
	m_boneLength = endN;
	m_boneLengthSquared = m_boneLength.x * m_boneLength.x + 
		m_boneLength.y * m_boneLength.y + m_boneLength.z * m_boneLength.z;
	endN.normalize();
	m_endPs.push_back(m_invBindTF * endP);
	m_endNs.push_back(m_invBindTF * endN);
}

void MayaHRBF::addVertex(MPoint pos, MVector nor) {
	// take pos and normal into local coordinates of this bone
	MPoint posL = m_invBindTF * pos;

	/***** cull lousy points, samples too close together (equation 2). *****/
	// presumably maya knows this is a dot
	double eq2 = (posL - m_bindPositionLocal) * m_boneLength / m_boneLengthSquared;

	if (eq2 > CULL_H && eq2 < 1.0 - CULL_H) {
		// put er in
		m_positions.push_back(posL);
		m_normals.push_back(m_invBindTF * nor);
	}
}

void MayaHRBF::compute() {

	/***** compute AABB. pad out to + 1/32 on each side. build grids *****/
	int numPositions = m_positions.size();
	float minX, minY, minZ, maxX, maxY, maxZ;
	minX = HUGE_VAL; maxX = -HUGE_VAL;
	minY = HUGE_VAL; maxY = -HUGE_VAL;
	minZ = HUGE_VAL; maxZ = -HUGE_VAL;
	MPoint pos;
	for (int i = 0; i < numPositions; i++) {
		pos = m_positions[i];
		minX = std::min(minX, (float)pos.x);
		minY = std::min(minY, (float)pos.y);
		minZ = std::min(minZ, (float)pos.z);

		maxX = std::max(maxX, (float)pos.x);
		maxY = std::max(maxY, (float)pos.y);
		maxZ = std::max(maxZ, (float)pos.z);
	}

	int numBonyLandmarks = m_endPs.size();
	for (int i = 0; i < numBonyLandmarks; i++) {
		pos = m_endPs[i];
		minX = std::min(minX, (float)pos.x);
		minY = std::min(minY, (float)pos.y);
		minZ = std::min(minZ, (float)pos.z);

		maxX = std::max(maxX, (float)pos.x);
		maxY = std::max(maxY, (float)pos.y);
		maxZ = std::max(maxZ, (float)pos.z);
	}

	float padX = (maxX - minX) / (float)HRBF_RES;
	float padY = (maxY - minY) / (float)HRBF_RES;
	float padZ = (maxZ - minZ) / (float)HRBF_RES;

	f_vals = new FloatGrid3D(HRBF_RES, HRBF_RES, HRBF_RES,
		minX - padX, minY - padY, minZ - padZ,
		maxX + padX, maxY + padY, maxZ + padZ);

	f_gradX = new FloatGrid3D(HRBF_RES, HRBF_RES, HRBF_RES,
		minX - padX, minY - padY, minZ - padZ,
		maxX + padX, maxY + padY, maxZ + padZ);

	f_gradY = new FloatGrid3D(HRBF_RES, HRBF_RES, HRBF_RES,
		minX - padX, minY - padY, minZ - padZ,
		maxX + padX, maxY + padY, maxZ + padZ);

	f_gradZ = new FloatGrid3D(HRBF_RES, HRBF_RES, HRBF_RES,
		minX - padX, minY - padY, minZ - padZ,
		maxX + padX, maxY + padY, maxZ + padZ);

	f_gradMag = new FloatGrid3D(HRBF_RES, HRBF_RES, HRBF_RES,
		minX - padX, minY - padY, minZ - padZ,
		maxX + padX, maxY + padY, maxZ + padZ);

	/***** compute unknowns (equation 1/vaillant's HRBF resources) *****/

	/***** compute HRBF values for every cell in the grids *****/
	// - don't forget to reparameterize
	// - also do the gradients
	// - see Vaillant's resources
	
}