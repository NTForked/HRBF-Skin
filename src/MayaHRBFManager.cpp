#include "MayaHRBFManager.h"

MayaHRBFManager::MayaHRBFManager() {
	mf_vals = new FloatGrid3D(HRBF_COMPRES, HRBF_COMPRES, HRBF_COMPRES,
		-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f);
	mf_gradX = new FloatGrid3D(HRBF_COMPRES, HRBF_COMPRES, HRBF_COMPRES,
		-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f);
	mf_gradY = new FloatGrid3D(HRBF_COMPRES, HRBF_COMPRES, HRBF_COMPRES,
		-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f);
	mf_gradZ = new FloatGrid3D(HRBF_COMPRES, HRBF_COMPRES, HRBF_COMPRES,
		-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f);
	mf_gradMag = new FloatGrid3D(HRBF_COMPRES, HRBF_COMPRES, HRBF_COMPRES,
		-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f);

}

MayaHRBFManager::~MayaHRBFManager() {
	if (mf_vals != NULL) delete mf_vals;
	if (mf_gradX != NULL) delete mf_gradX;
	if (mf_gradY != NULL) delete mf_gradY;
	if (mf_gradZ != NULL) delete mf_gradZ;
	if (mf_gradMag != NULL) delete mf_gradMag;

	// wipe whatever used to be in the hierarchy
	for (int i = 0; i < m_numJoints; i++) {
		if (m_HRBFs.at(i) != NULL)
			delete m_HRBFs.at(i);
	}
	m_HRBFs.clear();
}

void MayaHRBFManager::buildHRBFs(std::vector<int> jointHierarchy, std::vector<std::string> names,
	MMatrixArray &binds, MMatrixArray &transforms,
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
	m_numISOs = 0;

	for (; !iter.isDone(); iter.next()) {
		pt = iter.position();
		nor = iter.normal();
		hrbfCandidates.clear();
		candidateWeights.clear();
		numCandidates = 0;
		m_numISOs++;

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
		if (bestHRBF >= 0 && bestHRBF < m_numJoints)
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

	std::cout << "computing iso values..." << std::endl;
	// compute isovalues
	compose(transforms);
	m_isoVals.resize(m_numISOs);

	iter.reset();
	float iso;
	
	// additional statistics
	float iso_min = HUGE_VAL;
	float iso_max = -HUGE_VAL;
	float iso_avg = 0.0f;
	
	int i = 0;
	for (; !iter.isDone(); iter.next()) {
		pt = iter.position();
		mf_vals->trilinear(pt.x, pt.y, pt.z, iso);
		m_isoVals[i] = iso;
		i++;
		iso_avg += iso;
		iso_min = std::min(iso_min, iso);
		iso_max = std::max(iso_max, iso);
	}
	std::cout << "iso max: " << iso_max << std::endl;
	std::cout << "iso min: " << iso_min << std::endl;
	std::cout << "iso avg: " << iso_avg / (float)m_numISOs << std::endl;
}

void MayaHRBFManager::compose(MMatrixArray &transforms) {
	// compute matrix inverses for later
	//MMatrixArray inv;
	//for (int i = 0; i < numTransforms; i++) {
	//	inv.append(transforms[i].inverse());
	//}
	// compute an AABB around all the HRBFs
	// - for each HRBF grid, transform its 8 corners into global coordinates
	// - walk over all the transformed coordinates to get an HRBF value

	MPoint aabbMin(HUGE_VAL, HUGE_VAL, HUGE_VAL, 1.0);
	MPoint aabbMax(-HUGE_VAL, -HUGE_VAL, -HUGE_VAL, 1.0);
	MPoint minLocal;
	MPoint maxLocal;
	//std::string lowestName;
	std::vector<MPoint> mins;
	std::vector<MPoint> maxs;
	mins.resize(m_numJoints);
	maxs.resize(m_numJoints);


	for (int i = 0; i < m_numJoints; i++) {
		if (m_HRBFs[i]->m_posSamples.size() < 1) continue;
		//if (m_HRBFs[i]->m_name != "LeftLeg" && m_HRBFs[i]->m_name != "RightLeg") continue; // debug
		m_HRBFs[i]->mf_vals->getWorldAABB(transforms[i], minLocal, maxLocal);
		//if (aabbMin.y > minLocal.y) {
		//	lowestName = m_HRBFs[i]->m_name;
		//}
		mins[i] = minLocal;
		maxs[i] = maxLocal;

		aabbMin.x = std::min(aabbMin.x, minLocal.x);
		aabbMin.y = std::min(aabbMin.y, minLocal.y);
		aabbMin.z = std::min(aabbMin.z, minLocal.z);

		aabbMax.x = std::max(aabbMax.x, maxLocal.x);
		aabbMax.y = std::max(aabbMax.y, maxLocal.y);
		aabbMax.z = std::max(aabbMax.z, maxLocal.z);
	}
	// resize the grids, with a little padding
	double padx = 2.0 * (aabbMax.x - aabbMin.x) / (double)HRBF_COMPRES;
	double pady = 2.0 * (aabbMax.y - aabbMin.y) / (double)HRBF_COMPRES;
	double padz = 2.0 * (aabbMax.z - aabbMin.z) / (double)HRBF_COMPRES;

	aabbMax.x += padx;
	aabbMax.y += pady;
	aabbMax.z += padz;

	aabbMin.x -= padx;
	aabbMin.y -= pady;
	aabbMin.z -= padz;

	mf_vals->resizeAABB(aabbMin, aabbMax);
	mf_gradX->resizeAABB(aabbMin, aabbMax);
	mf_gradY->resizeAABB(aabbMin, aabbMax);
	mf_gradZ->resizeAABB(aabbMin, aabbMax);
	mf_gradMag->resizeAABB(aabbMin, aabbMax);

	// debug bbox
	//std::cout << "composed HRBF bbox is bounded by ";
	//std::cout << "max:" << aabbMax.x << " " << aabbMax.y << " " << aabbMax.z;
	//std::cout << " min:" << aabbMin.x << " " << aabbMin.y << " " << aabbMin.z << std::endl;;

	mf_vals->clear(0.0f);
	mf_gradX->clear(0.0f);
	mf_gradY->clear(0.0f);
	mf_gradZ->clear(0.0f);
	mf_gradMag->clear(0.0f);

	// compose the HRBFs
	// -for each grid in the manager, do:
	//  	for each point in the grid, do:
	//  		-transform point to world coordinates
	//  		-compute nearest cell in global grid
	//  		-query grid for value at that cell coordinate in local space
	//			-max onto grid

	/*********** splat ****************/
	// for each grid in the manager, compute its AABB in world space
	// then march over every cube at these indices in world space and compute interp val

	for (int i = 0; i < m_numJoints; i++) {
		MayaHRBF *grid = m_HRBFs[i];
		MMatrix toLocal = transforms[i].inverse();
		// compute cell indices of world AABB
		int x0, y0, z0;
		int x1, y1, z1;
		MPoint min = mins[i];
		MPoint max = maxs[i];
		mf_vals->coordToIDX(min.x, min.y, min.z, x0, y0, z0);
		mf_vals->coordToIDX(max.x, max.y, max.z, x1, y1, z1);

		// walk over all those cell points and sample at each one
		for (int x = x0; x < x1; x++) {
			for (int y = y0; y < y1; y++) {
				//#pragma omp parallel for // doesn't seem to help much
				float val;
				float dx;
				float dy;
				float dz;
				float mag;
				float currVal;
				float currMag;
				MPoint worldCoord;
				MPoint localCoord;
				for (int z = z0; z < z1; z++) {
					// transform coordinate into local space
					worldCoord = mf_vals->idxToCoord(x, y, z);
					localCoord = worldCoord * toLocal;
					grid->query(localCoord, val, dx, dy, dz, mag);

					currVal = mf_vals->getCell(x, y, z);
					currMag = mf_gradMag->getCell(x, y, z);
					if (val > currVal) {
						mf_vals->setCell(x, y, z, val);
					}
					if (mag > currMag) {
						mf_gradMag->setCell(x, y, z, mag);
						mf_gradX->setCell(x, y, z, dx);
						mf_gradY->setCell(x, y, z, dy);
						mf_gradZ->setCell(x, y, z, dz);
					}
				}
			}
		}
	}
	return;
}

void MayaHRBFManager::correct(MItGeometry& iter) {

	int i = 0;
	MPoint pt;
	MVector norm;

	float iso = 0.0f;
	float fv = 0.0f;
	MVector dfv(0.0, 0.0, 0.0);
	float dfx = 0.0f;
	float dfy = 0.0f;
	float dfz = 0.0f;
	MVector dfv_lag(0.0, 0.0, 0.0);
	float dfv_mag = 0.0f;

	for (; !iter.isDone(); iter.next()) {
		pt = iter.position();
		norm = iter.normal();
		iso = m_isoVals[i];
		norm.normalize();
		float iso_lag = iso;

		// do newton iterations - equation 4, but... with some modifications
		for (int j = 0; j < NEWTON_STEPS; j++) {
			mf_vals->trilinear(pt.x, pt.y, pt.z, fv);
			mf_gradX->trilinear(pt.x, pt.y, pt.z, dfx);
			mf_gradZ->trilinear(pt.x, pt.y, pt.z, dfy);
			mf_gradY->trilinear(pt.x, pt.y, pt.z, dfz);
			mf_gradMag->trilinear(pt.x, pt.y, pt.z, dfv_mag);
			dfv.x = dfx; dfv.y = dfy; dfv.z = dfz;
			// check for discontinuity: angle between grad in this step and previous
			// if angle is greater than 55 degrees, stop
			// aka if dot product is < COS_GRAD_ANGLE
			if (dfv.length() > 0.0001 && dfv_lag.length() > 0.0001
				&& j != 0 && dfv_lag.normal() * dfv.normal() < COS_GRAD_ANGLE) {
				//std::cout << "discontinuity on iteration " << j << std::endl;
				//std::cout << pt.x << " " << pt.y << " " << pt.z << std::endl;
				break;
			}
			if (j != 0 && abs(iso_lag - iso) < abs(iso - fv)) {
				//std::cout << "worsened on iteration " << j << std::endl;
			
				break; // quit if moving here is making your iso worse
			}
			iso_lag = fv;

			dfv_lag = dfv;
			
			pt += NEWTON_SIGMA * (iso - fv) * norm;// dfv.normal();// / (dfv_mag * dfv_mag); // unreliable for bad cases
		}
		iter.setPosition(pt);
		i++;
	}
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
	compose(transforms);
	mf_vals->exportToDebugString("composed HRBF");
}