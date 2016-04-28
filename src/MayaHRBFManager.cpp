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

void MayaHRBFManager::buldHRBFs(std::vector<int> jointHierarchy, std::vector<std::string> names,
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

	// compute isovalues
	compose(transforms, m_numJoints);
}

void MayaHRBFManager::compose(MMatrixArray &transforms, int numTransforms) {
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

	for (int i = 0; i < numTransforms; i++) {
		m_HRBFs[i]->mf_vals->getWorldAABB(transforms[i], minLocal, maxLocal);
		aabbMin.x = std::min(aabbMin.x, minLocal.x);
		aabbMin.y = std::min(aabbMin.y, minLocal.y);
		aabbMin.z = std::min(aabbMin.z, minLocal.z);

		aabbMax.x = std::max(aabbMax.x, maxLocal.x);
		aabbMax.y = std::max(aabbMax.y, maxLocal.y);
		aabbMax.z = std::max(aabbMax.z, maxLocal.z);
	}
	// resize the grids, with a little padding
	double padx = aabbMax.x - aabbMin.x / (double)HRBF_COMPRES;
	double pady = aabbMax.y - aabbMin.y / (double)HRBF_COMPRES;
	double padz = aabbMax.z - aabbMin.z / (double)HRBF_COMPRES;

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
	std::cout << "composed HRBF bbox is bounded by ";
	std::cout << "max:" << aabbMax.x << " " << aabbMax.y << " " << aabbMax.z;
	std::cout << " min:" << aabbMin.x << " " << aabbMin.y << " " << aabbMin.z << std::endl;;

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
	MayaHRBF *grid;
	MMatrix toWorld;
	MMatrix toLocal;
	float fx, fy, fz;
	MPoint localP1; // local space coordinates
	MPoint worldP1;
	MPoint worldP2; // nearest cell coordinate on composed grid
	MPoint localP2;
	float val, mag;
	int ix, iy, iz; // on the global grid
	float currVal;

	for (int i = 0; i < numTransforms; i++) {
		grid = m_HRBFs[i];
		toWorld = transforms[i];
		toLocal = toWorld.inverse();
		for (int x = 0; x < HRBF_RES; x++) {
			for (int y = 0; y < HRBF_RES; y++) {
				for (int z = 0; z < HRBF_RES; z++) {
					grid->mf_vals->idxToCoord(x, y, z, fx, fy, fz);
					localP1.x = fx; localP1.y = fy; localP1.z = fz; localP1.w = 1.0;
					// transform point to world coordinates
					worldP1 = localP1 * toWorld;
					// get nearest coordinate on the world grid
					mf_vals->coordToIDX(worldP1.x, worldP1.y, worldP1.z, ix, iy, iz);
					mf_vals->idxToCoord(ix, iy, iz, fx, fy, fz);
					worldP2.x = fx; worldP2.y = fy; worldP2.z = fz; worldP2.w = 1.0;
					// query grid for value at world grid's cell coordinate
					localP2 = worldP2 * toLocal;
					grid->query(localP2, val, fx, fy, fz, mag);
					// max onto grid
					currVal = mf_vals->getCell(ix, iy, iz);
					if (currVal < val) {
						mf_vals->setByCoordinate(ix, iy, iz, val);
						mf_gradX->setByCoordinate(ix, iy, iz, fx);
						mf_gradY->setByCoordinate(ix, iy, iz, fy);
						mf_gradZ->setByCoordinate(ix, iy, iz, fz);
						mf_gradMag->setByCoordinate(ix, iy, iz, mag);
					}
				}
			}
		}
	}
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
	mf_vals->exportToDebugString("composed HRBF");
}