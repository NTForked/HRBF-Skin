#include "HRBFNode.h"

void* HRBFSkinCluster::creator()
{
	HRBFSkinCluster *cluster = new HRBFSkinCluster();
	cluster->rebuildHRBFStatus = -1;
	return cluster;
}

// add some additional inputs
MStatus HRBFSkinCluster::initialize()
{
	std::cout << "called initialize" << std::endl;
	MFnNumericAttribute numAttr;

	MStatus returnStatus;
	HRBFSkinCluster::rebuildHRBF = numAttr.create("RebuildHRBF", "rbld", MFnNumericData::kInt,
		1, &returnStatus);
	McheckErr(returnStatus, "ERROR creating rbld attribute\n");
	returnStatus = addAttribute(HRBFSkinCluster::rebuildHRBF);
	McheckErr(returnStatus, "ERROR adding rbld attribute\n");

	HRBFSkinCluster::useDQ = numAttr.create("UseDualQuaternions", "useDQ", MFnNumericData::kInt,
		0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating useDQ attribute\n");
	returnStatus = addAttribute(HRBFSkinCluster::useDQ);
	McheckErr(returnStatus, "ERROR adding useDQ attribute\n");

	// set up affects
	returnStatus = attributeAffects(HRBFSkinCluster::rebuildHRBF,
		HRBFSkinCluster::outputGeom);
	McheckErr(returnStatus, "ERROR in attributeAffects with rebuildHRBF\n");

	// set up affects
	returnStatus = attributeAffects(HRBFSkinCluster::useDQ,
		HRBFSkinCluster::outputGeom);
	McheckErr(returnStatus, "ERROR in attributeAffects with useDQ\n");

    return returnStatus;
}

MStatus
HRBFSkinCluster::deform( MDataBlock& block,
                      MItGeometry& iter,
                      const MMatrix& m,
                      unsigned int multiIndex)
//
// Method: deform
//
// Description:   Deforms the point with a simple smooth skinning algorithm
//
// Arguments:
//   block      : the datablock of the node
//   iter       : an iterator for the geometry to be deformed
//   m          : matrix to transform the point into world space
//   multiIndex : the index of the geometry that we are deforming
//
//
{
	MStatus returnStatus;

	// get HRBF status
	MDataHandle HRBFstatusData = block.inputValue(rebuildHRBF, &returnStatus);
	McheckErr(returnStatus, "Error getting rebuildHRBF handle\n");
	int rebuildHRBFStatusNow = HRBFstatusData.asInt();

	// get skinning type
	MDataHandle useDQData = block.inputValue(useDQ, &returnStatus);
	McheckErr(returnStatus, "Error getting useDQ handle\n");
	int useDQNow = useDQData.asInt();

	// get envelope because why not
	MDataHandle envData = block.inputValue(envelope, &returnStatus);
	float env = envData.asFloat();

	// get the influence transforms
	//
	MArrayDataHandle transformsHandle = block.inputArrayValue( matrix ); // tell block what we want
	int numTransforms = transformsHandle.elementCount();
	if ( numTransforms == 0 ) { // no transforms, no problems
		return MS::kSuccess;
	}
	MMatrixArray transforms; // fetch transform matrices -> actual joint matrices
	for ( int i=0; i<numTransforms; ++i ) {
		transforms.append( MFnMatrixData( transformsHandle.inputValue().data() ).matrix() );
		transformsHandle.next();
	}
	// inclusive matrices inverse of the driving transform at time of bind
	// matrices for transforming vertices to joint local space
	MArrayDataHandle bindHandle = block.inputArrayValue( bindPreMatrix ); // tell block what we want
	if ( bindHandle.elementCount() > 0 ) {
		for ( int i=0; i<numTransforms; ++i ) {
			transforms[i] = MFnMatrixData( bindHandle.inputValue().data() ).matrix() * transforms[i];
			bindHandle.next();
		}
	}

	MArrayDataHandle weightListHandle = block.inputArrayValue(weightList);
	if (weightListHandle.elementCount() == 0) {
		// no weights - nothing to do
		std::cout << "no weights!" << std::endl;
		rebuildHRBFStatus = rebuildHRBFStatusNow - 1; // HRBFs will need to rebuilt no matter what
		return MS::kSuccess;
	}

	// perform traditional skinning
	if (useDQNow != 0) {
		returnStatus = skinDQ(transforms, numTransforms, weightListHandle, iter);
	}
	else {
		returnStatus = skinLB(transforms, numTransforms, weightListHandle, iter);
	}

	// check if HRBFs need to be recomputed
	if (rebuildHRBFStatus != rebuildHRBFStatusNow) {
		std::cout << "instructed to rebuild HRBFs" << std::endl;
		rebuildHRBFStatus = rebuildHRBFStatusNow;
	}

	// do HRBF corrections

	return returnStatus;
}

MStatus
HRBFSkinCluster::skinDQ(MMatrixArray&  transforms,
						int numTransforms,
						MArrayDataHandle& weightListHandle,
						MItGeometry& iter) {
	MStatus returnStatus;

	// compute dual quaternions. we're storing them as a parallel array.
	std::vector<MQuaternion> tQuaternions(numTransforms); // translation quaterions
	std::vector<MQuaternion> rQuaternions(numTransforms); // rotation quaternions

	for (int i = 0; i < numTransforms; i++) {
		rQuaternions.at(i) = getRotationQuaternion(transforms[i]);
		rQuaternions.at(i).normalizeIt();
		tQuaternions.at(i) = getTranslationQuaternion(transforms[i], rQuaternions.at(i));
#if DEBUG_PRINTS
		std::cout << "rota quaternion " << i << " is: " << rQuaternions.at(i) << std::endl;
		std::cout << "tran quaternion " << i << " is: " << tQuaternions.at(i) << std::endl;
#endif
	}

	MQuaternion rBlend; // blended rotation quaternions
	MQuaternion tBlend; // blended translation quaternions
	MQuaternion scaleMe; // Maya's quaternion scaling is in-place
	double weight;


	// Iterate through each point in the geometry.
	//
	for (; !iter.isDone(); iter.next()) {
		MPoint pt = iter.position();
		MPoint skinned;

		rBlend = MQuaternion(); // reset
		tBlend = MQuaternion(); // reset
		rBlend[3] = 0.0;
		tBlend[3] = 0.0;

		// get the weights for this point
		MArrayDataHandle weightsHandle = weightListHandle.inputValue().child(weights);

		// compute the skinning
		for (int i = 0; i<numTransforms; ++i) {
			if (MS::kSuccess == weightsHandle.jumpToElement(i)) {
				weight = weightsHandle.inputValue().asDouble();
				scaleMe = rQuaternions.at(i);
				rBlend = rBlend + scaleMe.scaleIt(weight);
				scaleMe = tQuaternions.at(i);
				tBlend = tBlend + scaleMe.scaleIt(weight);
			}
		}

		MMatrix dqMatrix = makeDQMatrix(rBlend.normalizeIt(), tBlend);
		skinned = pt * dqMatrix;

		// Set the final position.
		iter.setPosition(skinned);

		// advance the weight list handle
		weightListHandle.next();
	}
	return returnStatus;
}
MStatus HRBFSkinCluster::skinLB(MMatrixArray&  transforms,
								int numTransforms,
								MArrayDataHandle& weightListHandle,
								MItGeometry& iter) {
	MStatus returnStatus;

	// Iterate through each point in the geometry.
	//
	for (; !iter.isDone(); iter.next()) {
		MPoint pt = iter.position();
		MPoint skinned;
		// get the weights for this point
		MArrayDataHandle weightsHandle = weightListHandle.inputValue().child(weights);
		// compute the skinning -> TODO: what's the order that the weights are given in?
		for (int i = 0; i<numTransforms; ++i) {
			if (MS::kSuccess == weightsHandle.jumpToElement(i)) {
				skinned += (pt * transforms[i]) * weightsHandle.inputValue().asDouble();
			}
		}

		// Set the final position.
		iter.setPosition(skinned);
		// advance the weight list handle
		weightListHandle.next();
	}
	return returnStatus;
}


// standard initialization procedures
//

MStatus initializePlugin( MObject obj )
{
    MStatus result;

    MFnPlugin plugin( obj, PLUGIN_COMPANY, "3.0", "Any");
	std::cout << "initializing HRBF plugin" << std::endl;
    result = plugin.registerNode(
        "HRBFSkinCluster" ,
        HRBFSkinCluster::id ,
        &HRBFSkinCluster::creator ,
        &HRBFSkinCluster::initialize ,
        MPxNode::kSkinCluster
        );

    return result;
}

MStatus uninitializePlugin( MObject obj )
{
    MStatus result;

    MFnPlugin plugin( obj );
    result = plugin.deregisterNode( HRBFSkinCluster::id );

    return result;
}
