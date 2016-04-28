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

	mf_vals = new FloatGrid3D(HRBF_RES, HRBF_RES, HRBF_RES, 
		-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f);
	mf_gradX = new FloatGrid3D(HRBF_RES, HRBF_RES, HRBF_RES,
		-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f);
	mf_gradY = new FloatGrid3D(HRBF_RES, HRBF_RES, HRBF_RES,
		-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f);
	mf_gradZ = new FloatGrid3D(HRBF_RES, HRBF_RES, HRBF_RES,
		-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f);
	mf_gradMag = new FloatGrid3D(HRBF_RES, HRBF_RES, HRBF_RES,
		-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f);

	m_r = -1.0f;

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
	//MPoint p3 = p1 * invBindTF;
	//MPoint p4 = p3 * m_bindTF;


	return;
}

MayaHRBF::~MayaHRBF() {
	if (mf_vals != NULL) delete mf_vals;
	if (mf_gradX != NULL) delete mf_gradX;
	if (mf_gradY != NULL) delete mf_gradY;
	if (mf_gradZ != NULL) delete mf_gradZ;
	if (mf_gradMag != NULL) delete mf_gradMag;

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

float compact(float f, float r) {
	if (f < -r) return 1.0f;
	if (f > r) return 0.0f;
	// (-3/16)*(f/radius)^5+(5/8)*(f/radius)^3-(15/16)*(f/radius) + 1/2
	float f_div_r = f / r;
	float f_div_r2 = f_div_r * f_div_r;
	float f_div_r4 = f_div_r2 * f_div_r2;
	return (-3.0f / 16.0f) * f_div_r4 * f_div_r + (5.0f / 8.0f) * f_div_r2 * f_div_r - (15.0f / 16.0f) * f_div_r + 0.5f;
}

void dcompact(float f, float r, float &dx, float &dy, float &dz) {
	if (f < -r || f > r) {
		dx = 0.0f;
		dy = 0.0f;
		dz = 0.0f;
		return;
	}
	// (-15/(16r))*(f/r)^4 + (15/(8*r))*(f/r)^2 - (15/(16r))
	float f_div_r = f / r;
	float f_div_r2 = f_div_r * f_div_r;
	float f_div_r4 = f_div_r2 * f_div_r2;
	float tmp = (-15.0f / (16.0f*r));
	float scale = tmp * f_div_r4 + (15.0f / 8.0f * r) * f_div_r2 + tmp;
	dx *= scale;
	dy *= scale;
	dz *= scale;
}

void MayaHRBF::compute() {

	// don't do anything if there are no sample points. happens sometimes.
	int numSamples = m_posSamples.size();
	int numExtremities = m_posExtrem.size();

	if (numSamples == 0) return;

	/***** compute extremity closing samples *****/
	closeExtremities();

	/***** compute reparameterization radius R *****/
	// - if there are only two bones, do point-line distance
	// - if there are more bones, do distance to the root bone
	// http://paulbourke.net/geometry/pointlineplane/
	if (m_numChildren == 1) {
		float u;
		MPoint P1 = m_jointPositionLocal;
		MPoint P2 = m_jointPosLocals[0];
		MPoint P3;
		MVector boneLength = P2 - P1;
		MPoint nearestPt;
		float boneLength2 = boneLength.length();
		boneLength2 *= boneLength2;
		float candidate;
		for (int i = 0; i < numSamples; i++) {
			P3 = m_posSamples[i];
			u = (P3.x - P1.x) * boneLength.x + (P3.y - P1.y) * boneLength.y + (P3.z - P1.z) * boneLength.z;
			u /= boneLength2;
			u = std::min(std::max(u, 0.0f), 1.0f); // clamp
			nearestPt = P1 + boneLength * u;
			m_r = std::max(m_r, (float)(nearestPt - P3).length());
		}
	}
	else {
		for (int i = 0; i < numSamples; i++) {
			m_r = std::max(m_r, (float)(m_posSamples.at(i) - m_jointPositionLocal).length());
		}
	}

	/***** compute AABB. pad out to + r on each side. build grids *****/
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

	float pad = m_r;

	mf_vals->resizeAABB(
		minX - pad, minY - pad, minZ - pad,
		maxX + pad, maxY + pad, maxZ + pad);
	mf_vals->clear(0.0f);

	mf_gradX->resizeAABB(
		minX - pad, minY - pad, minZ - pad,
		maxX + pad, maxY + pad, maxZ + pad);
	mf_gradX->clear(0.0f);

	mf_gradY->resizeAABB(
		minX - pad, minY - pad, minZ - pad,
		maxX + pad, maxY + pad, maxZ + pad);
	mf_gradY->clear(0.0f);

	mf_gradZ->resizeAABB(
		minX - pad, minY - pad, minZ - pad,
		maxX + pad, maxY + pad, maxZ + pad);
	mf_gradZ->clear(0.0f);

	mf_gradMag->resizeAABB(
		minX - pad, minY - pad, minZ - pad,
		maxX + pad, maxY + pad, maxZ + pad);
	mf_gradMag->clear(0.0f);


	//mf_vals->clear(1.0f);
	//return; // debug: eigen is too slow in debug mode for this to be bearable

	/***** compute unknowns (equation 1/vaillant's HRBF resources) *****/
	reduceSamples();
	numSamples = m_posSamples.size();
	std::vector<MVector> positions;
	for (int i = 0; i < numSamples; i++) {
		positions.push_back(m_posSamples[i]); // push back as MVector
	}

	std::vector<MVector> normals(m_norSamples);

	for (int i = 0; i < numExtremities; i++) {
		positions.push_back(m_posExtrem[i]);
		normals.push_back(m_norExtrem[i]);
	}
	HRBF3 hrbf(positions, normals); // set up hrbf

	/***** compute HRBF values for every cell in the grids *****/
	// - reparameterize - see Vaillant's resources
	// - also do the gradients

	for (int x = 0; x < HRBF_RES; x++) {
		#pragma omp parallel for
		for (int y = 0; y < HRBF_RES; y++) {
			float xf, yf, zf;
			float dx, dy, dz;
			float val;
			float mag;
			for (int z = 0; z < HRBF_RES; z++) {
				// get coordinates
				mf_vals->idxToCoord(x, y, z, xf, yf, zf);

				// plug into HRBF solver
				val = hrbf.evaluate(xf, yf, zf);
				hrbf.gradient(xf, yf, zf, dx, dy, dz);

				// reparameterize
				val = compact(val, m_r);
				dcompact(val, m_r, dx, dy, dz);

				// shove into the grids
				mf_vals->setCell(x, y, z, val);
				if (val > 0.00001) {
					mf_gradX->setCell(x, y, z, dx);
					mf_gradY->setCell(x, y, z, dy);
					mf_gradZ->setCell(x, y, z, dz);
					mf_gradMag->setCell(x, y, z, sqrt(dx * dx * dz * dz * dy * dy));
				}
			}
		}
	}
}

void MayaHRBF::query(float x, float y, float z,
	float &val, float &gradX, float &gradY, float &gradZ, float &gradM) {
	if (mf_vals->checkBounds(x, y, z)) {
		mf_vals->trilinear(x, y, z, val);
		mf_gradX->trilinear(x, y, z, gradX);
		mf_gradY->trilinear(x, y, z, gradY);
		mf_gradZ->trilinear(x, y, z, gradZ);
		mf_gradMag->trilinear(x, y, z, gradM);
	}
	else {
		val = 0.0f;
		gradX = 0.0f;
		gradY = 0.0f;
		gradZ = 0.0f;
		gradM = 0.0f;
	}
}

void MayaHRBF::query(MPoint pos, float &val, float &gradX, float &gradY, float &gradZ, float &gradM) {
	if (mf_vals->checkBounds(pos.x, pos.y, pos.z)) {
		mf_vals->trilinear(pos.x, pos.y, pos.z, val);
		mf_gradX->trilinear(pos.x, pos.y, pos.z, gradX);
		mf_gradY->trilinear(pos.x, pos.y, pos.z, gradY);
		mf_gradZ->trilinear(pos.x, pos.y, pos.z, gradZ);
		mf_gradMag->trilinear(pos.x, pos.y, pos.z, gradM);
		//val = mf_vals->getByCoordinate(pos.x, pos.y, pos.z);
		//gradX = mf_gradX->getByCoordinate(pos.x, pos.y, pos.z);
		//gradY = mf_gradY->getByCoordinate(pos.x, pos.y, pos.z);
		//gradZ = mf_gradZ->getByCoordinate(pos.x, pos.y, pos.z);
		//gradM = mf_gradMag->getByCoordinate(pos.x, pos.y, pos.z);
	}
	else {
		val = 0.0f;
		gradX = 0.0f;
		gradY = 0.0f;
		gradZ = 0.0f;
		gradM = 0.0f;
	}
}

void MayaHRBF::printHRBFSamplingDebug() {
	float minX = mf_vals->m_min.x;
	float minY = mf_vals->m_min.y;
	float minZ = mf_vals->m_min.z;
	float maxX = mf_vals->m_max.x;
	float maxY = mf_vals->m_max.y;
	float maxZ = mf_vals->m_max.z;

	float norDispMag = MVector(maxX - minX, maxY - minY, maxZ - minZ).length() / (float)HRBF_RES;

	FloatGrid3D debugSplat = FloatGrid3D(HRBF_RES, HRBF_RES, HRBF_RES,
		minX - norDispMag, minY - norDispMag, minZ - norDispMag,
		maxX + norDispMag, maxY + norDispMag, maxZ + norDispMag);


	// splat all samples onto the debug grid
	int numSamples = m_posSamples.size();
	int numExtrems = m_posExtrem.size();

	MPoint sample;
	MPoint norDisp;
	debugSplat.clear(-1.0f);
	for (int i = 0; i < numSamples; i++) {
		sample = m_posSamples[i];
		norDisp = sample + m_norSamples[i] * norDispMag;
		if (!debugSplat.setByCoordinate(sample.x, sample.y, sample.z, 1.0)) {
			std::cout << "splat fail\n"; // shouldn't happen, but leave this for debugging anyway
		}
		if (!debugSplat.setByCoordinate(norDisp.x, norDisp.y, norDisp.z, 2.0)) {
			std::cout << "splat fail\n";
		}
	}

	for (int i = 0; i < numExtrems; i++) {
		sample = m_posExtrem[i];
		norDisp = sample + m_norExtrem[i] * norDispMag;
		if (!debugSplat.setByCoordinate(sample.x, sample.y, sample.z, 3.0)) {
			std::cout << "splat fail\n";
		}
		if (!debugSplat.setByCoordinate(norDisp.x, norDisp.y, norDisp.z, 4.0)) {
			std::cout << "splat fail\n";
		}
	}

	debugSplat.exportToDebugString(m_name);
}

void MayaHRBF::printHRBF() {
	mf_vals->exportToDebugString(m_name);
	//mf_gradMag->exportToDebugString(m_name);
}

void MayaHRBF::reduceSamples() {
	int numSamples = m_posSamples.size();
	if (numSamples < SAMPLE_CAP) return;
	
	// randomly remove samples until we get beneath the cap
	int randIDX;
	int numSamplesNow = numSamples;
	for (int i = 0; i < numSamples - SAMPLE_CAP; i++) {
		randIDX = rand() % numSamplesNow;
		m_posSamples.erase(m_posSamples.begin() + randIDX);
		m_norSamples.erase(m_norSamples.begin() + randIDX);
		numSamplesNow--;
	}
}