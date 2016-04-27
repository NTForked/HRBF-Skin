#include "MayaHRBF.h"

MayaHRBF::MayaHRBF(std::string &name, MMatrix &invBindTF) {
	m_parent = NULL;
	m_name = name;
	m_invBindTF = invBindTF;
	m_bindTF = invBindTF.inverse();

	m_jointPositionWorld.x = m_bindTF(3, 0);
	m_jointPositionWorld.y = m_bindTF(3, 1);
	m_jointPositionWorld.z = m_bindTF(3, 2);
	m_jointPositionLocal = m_jointPositionWorld * invBindTF;

	// some test things because idk what maya is even doing anymore
	//MMatrix IDhopefully1 = invBindTF * m_bindTF;
	//MMatrix IDhopefully2 = m_bindTF * invBindTF;
	//
	//MPoint p1(1.0, 1.0, 1.0);
	//MPoint p2(2.0, 2.0, 2.0);
	//MVector v1 = p2 - p1;
	//MVector v2 = p1 - p2;
	//MVector v1s1 = v1 * 2.0;
	//MVector v1s2 = 2.0 * v1;


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
	m_numChildren = m_children.size();
	MPoint endP;
	MVector endN;
	for (int i = 0; i < m_numChildren; i++) {
		endP = m_children.at(i)->m_jointPositionWorld * m_invBindTF;
		endN = endP - m_jointPositionLocal;
		endN.normalize();
		m_jointPosLocals.push_back(endP);
		m_jointDirLocals.push_back(endN);
	}
}

void MayaHRBF::addVertex(MPoint pos, MVector nor) {
	// take pos and normal into local coordinates of this bone
	MPoint posL = pos * m_invBindTF;

	/***** cull lousy points, samples too close to a joint (equation 2). *****/
	// cull for each pair root -> child
	// presumably maya knows this is a dot
	bool cull = false;
	for (int i = 0; i < m_numChildren; i++) {
		MVector m_dist = m_jointPosLocals[i] - m_jointPositionLocal;
		m_dist.x = abs(m_dist.x);
		m_dist.y = abs(m_dist.y);
		m_dist.z = abs(m_dist.z);
		double m_boneLengthSquared = m_dist.x * m_dist.x + m_dist.y * m_dist.y + m_dist.z * m_dist.z;
		MVector distVert = posL - m_jointPositionLocal;
		distVert.x = abs(distVert.x);
		distVert.y = abs(distVert.y);
		distVert.z = abs(distVert.z);

		double eq2 = distVert * m_dist / m_boneLengthSquared;
		if (CULL_H > eq2 || eq2 > 1.0 - CULL_H) {
			cull = true;
			break;
		}
	}

	if (!cull) {
		// put er in
		m_posSamples.push_back(posL);
		m_norSamples.push_back(nor * m_invBindTF);
	}
}

bool MayaHRBF::isExtremity() {
	// -has just one child
	// -or has a parent and dir_parent DOT child[i] is always < 0
	if (m_numChildren < 2) return true;
	if (m_parent == NULL) return false;
	// do a dot product with each child dir and the parent dir
	MVector dirToParent = m_invBindTF * m_parent->m_jointPositionWorld;
	dirToParent.normalize();
	for (int i = 0; i < m_numChildren; i++) {
		if (dirToParent * m_jointDirLocals[i] > 0.0) return false;
	}
	return true;
}

void MayaHRBF::closeExtremities() {
	/***** compute extremity closing samples *****/
	// don't do this if there's no other bones
	if (m_numChildren < 1) return;

	int numSamples = m_posSamples.size();
	double d;
	double cand; // candidate
	MPoint jointPos;
	for (int i = 0; i < m_numChildren; i++) {
		// find the nearest distance d in all the samples to this point
		d = HUGE_VAL;
		jointPos = m_jointPosLocals[i];
		for (int j = 0; j < numSamples; j++) {
			cand = (jointPos - m_posSamples[j]).length();
			d = std::min(cand, d);
		}
		// add a new point displaced by d * nor from this joint position
		m_posExtrem.push_back(jointPos + m_jointDirLocals[i] * d);
		m_norExtrem.push_back(m_jointDirLocals[i]);
	}
	// don't do one for this point unless it is a "confirmed extremity":
	if (isExtremity()) {
		// compute an averaged normal that points "away" from the child joints
		MPoint avgChildPos(0.0,0.0,0.0);
		for (int i = 0; i < m_numChildren; i++) {
			avgChildPos += m_jointPosLocals[i];
		}
		avgChildPos = avgChildPos / double(m_numChildren);
		MVector avgNor = m_jointPositionLocal - avgChildPos;
		avgNor.normalize();

		// find the  closest point
		d = HUGE_VAL;
		for (int j = 0; j < numSamples; j++) {
			cand = (m_jointPositionLocal - m_posSamples[j]).length();
			d = std::min(cand, d);
		}
		// compute the extremity and push
		m_posExtrem.push_back(m_jointPositionLocal + avgNor * d);
		m_norExtrem.push_back(avgNor);
	}
}

void MayaHRBF::compute() {

	/***** compute extremity closing samples *****/
	closeExtremities();

	/***** compute AABB. pad out to + 1/32 on each side. build grids *****/
	int numPositions = m_posSamples.size();
	float minX, minY, minZ, maxX, maxY, maxZ;
	minX = HUGE_VAL; maxX = -HUGE_VAL;
	minY = HUGE_VAL; maxY = -HUGE_VAL;
	minZ = HUGE_VAL; maxZ = -HUGE_VAL;
	MPoint pos;
	for (int i = 0; i < numPositions; i++) {
		pos = m_posSamples[i];
		minX = std::min(minX, (float)pos.x);
		minY = std::min(minY, (float)pos.y);
		minZ = std::min(minZ, (float)pos.z);

		maxX = std::max(maxX, (float)pos.x);
		maxY = std::max(maxY, (float)pos.y);
		maxZ = std::max(maxZ, (float)pos.z);
	}

	int numExtremitySamples = m_posExtrem.size();
	for (int i = 0; i < numExtremitySamples; i++) {
		pos = m_posExtrem[i];
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

void MayaHRBF::printHRBFSamplingDebug() {
	float minX = f_vals->m_min.x;
	float minY = f_vals->m_min.y;
	float minZ = f_vals->m_min.z;
	float maxX = f_vals->m_max.x;
	float maxY = f_vals->m_max.y;
	float maxZ = f_vals->m_max.z;

	float norDispMag = MVector(maxX - minX, maxY - minY, maxZ - minZ).length() / (float)HRBF_RES;

	FloatGrid3D debugSplat = FloatGrid3D(HRBF_RES, HRBF_RES, HRBF_RES,
		minX - norDispMag, minY - norDispMag, minZ - norDispMag,
		maxX + norDispMag, maxY + norDispMag, maxZ + norDispMag);


	// splat all samples onto the debug grid
	int numSamples = m_posSamples.size();
	int numExtrems = m_posExtrem.size();

	MPoint sample;
	MPoint norDisp;
	for (int i = 0; i < numSamples; i++) {
		sample = m_posSamples[i];
		norDisp = sample + m_norSamples[i] * norDispMag;
		debugSplat.setByCoordinate(sample.x, sample.y, sample.z, 1.0);
		debugSplat.setByCoordinate(norDisp.x, norDisp.y, norDisp.z, 2.0);
	}

	for (int i = 0; i < numExtrems; i++) {
		sample = m_posExtrem[i];
		norDisp = sample + m_norExtrem[i] * norDispMag;
		debugSplat.setByCoordinate(sample.x, sample.y, sample.z, 3.0);
		debugSplat.setByCoordinate(norDisp.x, norDisp.y, norDisp.z, 4.0);
	}

	// export the debug grid
	//std::cout << "exporting " << m_name.c_str() << std::endl;
	debugSplat.exportToDebugString(m_name);
}