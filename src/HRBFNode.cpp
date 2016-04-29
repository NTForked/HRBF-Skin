#include "HRBFNode.h"

void* HRBFSkinCluster::creator()
{
	HRBFSkinCluster *cluster = new HRBFSkinCluster();
	// a little big of HRBF setup:
	cluster->rebuildHRBFStatus = 0; // rebuild manually
	cluster->exportHRBFSamplesStatus = ""; // don't export normally
	cluster->exportHRBFValuesStatus = ""; // don't export normally

	cluster->exportCompositionStatus = 0; // default value. don't export unless asked.
	cluster->hrbfMan = MayaHRBFManager();
	return cluster;
}

// add some additional inputs
MStatus HRBFSkinCluster::initialize()
{
	std::cout << "called initialize" << std::endl;

	MFnNumericAttribute numAttr;
	MFnTypedAttribute typedAttr;
	//MFnCompoundAttribute cmpdAttr;

	MStatus returnStatus;
	HRBFSkinCluster::rebuildHRBF = numAttr.create("RebuildHRBF", "rbld", MFnNumericData::kInt,
		0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating rbld attribute\n");
	returnStatus = addAttribute(HRBFSkinCluster::rebuildHRBF);
	McheckErr(returnStatus, "ERROR adding rbld attribute\n");

	HRBFSkinCluster::exportComposition = numAttr.create("ExportComp", "exprtC", MFnNumericData::kInt,
		0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating exprtC attribute\n");
	returnStatus = addAttribute(HRBFSkinCluster::exportComposition);
	McheckErr(returnStatus, "ERROR adding exprtC attribute\n");

	HRBFSkinCluster::exportHRBFSamples = typedAttr.create("ExportHRBFs", "exprtS", MFnNumericData::kString,
		&returnStatus);
	McheckErr(returnStatus, "ERROR creating exprtS attribute\n");
	returnStatus = addAttribute(HRBFSkinCluster::exportHRBFSamples);
	McheckErr(returnStatus, "ERROR adding exprtS attribute\n");

	HRBFSkinCluster::exportHRBFValues = typedAttr.create("ExportHRBFv", "exprtV", MFnNumericData::kString,
		&returnStatus);
	McheckErr(returnStatus, "ERROR creating exprtV attribute\n");
	returnStatus = addAttribute(HRBFSkinCluster::exportHRBFValues);
	McheckErr(returnStatus, "ERROR adding exprtV attribute\n");

	HRBFSkinCluster::useDQ = numAttr.create("UseDualQuaternions", "useDQ", MFnNumericData::kInt,
		0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating useDQ attribute\n");
	returnStatus = addAttribute(HRBFSkinCluster::useDQ);
	McheckErr(returnStatus, "ERROR adding useDQ attribute\n");

	HRBFSkinCluster::useHRBF = numAttr.create("UseHRBFCorrection", "useHRBF", MFnNumericData::kInt,
		0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating useHRBF attribute\n");
	returnStatus = addAttribute(HRBFSkinCluster::useHRBF);
	McheckErr(returnStatus, "ERROR adding useHRBF attribute\n");

	HRBFSkinCluster::checkHRBFAt = numAttr.create("checkHRBFAt", "chkHRBF", MFnNumericData::k3Double,
		0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating checkHRBFAt attribute\n");
	returnStatus = addAttribute(HRBFSkinCluster::checkHRBFAt);
	McheckErr(returnStatus, "ERROR adding checkHRBFAt attribute\n");

	// hierarchy information
	// http://download.autodesk.com/us/maya/2011help/API/weight_list_node_8cpp-example.html#a15
	HRBFSkinCluster::jointParentIdcs = numAttr.create("parentJointIDCS", "pJIDCS", MFnNumericData::kInt,
		0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating pIDCS attribute\n");
	numAttr.setArray(true);
	returnStatus = addAttribute(HRBFSkinCluster::jointParentIdcs);
	McheckErr(returnStatus, "ERROR adding pIDCS attribute\n");

	// joint names
	HRBFSkinCluster::jointNames = typedAttr.create("jointNames", "jNms", MFnData::kString,
		&returnStatus);
	McheckErr(returnStatus, "ERROR creating pIDCS attribute\n");
	typedAttr.setArray(true);
	returnStatus = addAttribute(HRBFSkinCluster::jointNames);
	McheckErr(returnStatus, "ERROR adding jNms attribute\n");

	// set up affects
	returnStatus = attributeAffects(HRBFSkinCluster::rebuildHRBF,
		HRBFSkinCluster::outputGeom);
	McheckErr(returnStatus, "ERROR in attributeAffects with rebuildHRBF\n");

	returnStatus = attributeAffects(HRBFSkinCluster::exportComposition,
		HRBFSkinCluster::outputGeom);
	McheckErr(returnStatus, "ERROR in attributeAffects with exportComposition\n");

	returnStatus = attributeAffects(HRBFSkinCluster::exportHRBFSamples,
		HRBFSkinCluster::outputGeom);
	McheckErr(returnStatus, "ERROR in attributeAffects with exportHRBFSamples\n");

	returnStatus = attributeAffects(HRBFSkinCluster::exportHRBFValues,
		HRBFSkinCluster::outputGeom);
	McheckErr(returnStatus, "ERROR in attributeAffects with exportHRBFValues\n");

	returnStatus = attributeAffects(HRBFSkinCluster::useDQ,
		HRBFSkinCluster::outputGeom);
	McheckErr(returnStatus, "ERROR in attributeAffects with useDQ\n");

	returnStatus = attributeAffects(HRBFSkinCluster::useHRBF,
		HRBFSkinCluster::outputGeom);
	McheckErr(returnStatus, "ERROR in attributeAffects with useHRBF\n");

	returnStatus = attributeAffects(HRBFSkinCluster::jointParentIdcs,
		HRBFSkinCluster::outputGeom);
	McheckErr(returnStatus, "ERROR in attributeAffects with jointParentIdcs\n");

	returnStatus = attributeAffects(HRBFSkinCluster::checkHRBFAt,
		HRBFSkinCluster::outputGeom);
	McheckErr(returnStatus, "ERROR in attributeAffects with checkHRBFAt\n");

    return returnStatus;
}

MStatus
HRBFSkinCluster::deform( MDataBlock& block,
                      MItGeometry& iter,
                      const MMatrix& m,
                      unsigned int multiIndex)
//
// Method: deform1
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
	// handle signaling to the rest of deform that HRBFs must be rebuild
	bool signalRebuildHRBF = false;
	signalRebuildHRBF = (rebuildHRBFStatus != rebuildHRBFStatusNow);
	MMatrixArray bindTFs; // store just the bind transforms in here.
	MMatrixArray boneTFs; // ALWAYS store just the bone transforms in here.

	// get HRBF export status
	MDataHandle exportCompositionData = block.inputValue(exportComposition, &returnStatus);
	McheckErr(returnStatus, "Error getting exportComposition handle\n");
	int exportCompositionStatusNow = exportCompositionData.asInt();

	MDataHandle HRBFExportSamplesData = block.inputValue(exportHRBFSamples, &returnStatus);
	McheckErr(returnStatus, "Error getting exportHRBFSamples handle\n");
	std::string exportHRBFSamplesStatusNow = HRBFExportSamplesData.asString().asChar();

	MDataHandle HRBFExportValuesData = block.inputValue(exportHRBFValues, &returnStatus);
	McheckErr(returnStatus, "Error getting exportHRBFValues handle\n");
	std::string exportHRBFValuesStatusNow = HRBFExportValuesData.asString().asChar();

	// get skinning type
	MDataHandle useDQData = block.inputValue(useDQ, &returnStatus);
	McheckErr(returnStatus, "Error getting useDQ handle\n");
	int useDQNow = useDQData.asInt();

	// determine if we're using HRBF
	MDataHandle useHRBFData = block.inputValue(useHRBF, &returnStatus);
	McheckErr(returnStatus, "Error getting useHRBFData handle\n");
	int useHRBFnow = useHRBFData.asInt();

	// get envelope because why not
	MDataHandle envData = block.inputValue(envelope, &returnStatus);
	float env = envData.asFloat();

	// get point in space for evaluating HRBF
	MDataHandle checkHRBFAtData = block.inputValue(checkHRBFAt, &returnStatus);
	McheckErr(returnStatus, "Error getting useDQ handle\n");
	double* data = checkHRBFAtData.asDouble3();

	// get the influence transforms
	//
	MArrayDataHandle transformsHandle = block.inputArrayValue( matrix ); // tell block what we want
	int numTransforms = transformsHandle.elementCount();
	if ( numTransforms == 0 ) { // no transforms, no problems
		return MS::kSuccess;
	}
	MMatrixArray transforms; // fetch transform matrices -> actual joint matrices
	for ( int i=0; i<numTransforms; ++i ) {
		MMatrix worldTF = MFnMatrixData(transformsHandle.inputValue().data()).matrix();
		transforms.append(worldTF);
		boneTFs.append(worldTF);
		transformsHandle.next();
	}
	// inclusive matrices inverse of the driving transform at time of bind
	// matrices for transforming vertices to joint local space
	MArrayDataHandle bindHandle = block.inputArrayValue( bindPreMatrix ); // tell block what we want
	if ( bindHandle.elementCount() > 0 ) {
		for ( int i=0; i<numTransforms; ++i ) {
			MMatrix bind = MFnMatrixData(bindHandle.inputValue().data()).matrix();
			transforms[i] = bind * transforms[i];
			bindHandle.next();
			if (signalRebuildHRBF) bindTFs.append(bind);
		}
	}

	MArrayDataHandle weightListHandle = block.inputArrayValue(weightList);
	if (weightListHandle.elementCount() == 0) {
		// no weights - nothing to do
		std::cout << "no weights!" << std::endl;
		//rebuildHRBFStatus = rebuildHRBFStatusNow - 1; // HRBFs will need to rebuilt no matter what
		return MS::kSuccess;
	}

	// print HRBF samples if requested
	if (exportHRBFSamplesStatusNow != exportHRBFSamplesStatus) {
		std::cout << "instructed to export HRBF samples: " << exportHRBFSamplesStatusNow.c_str() << std::endl;
		exportHRBFSamplesStatus = exportHRBFSamplesStatusNow;
		// TODO: handle exporting HRBFs to the text file format
		hrbfMan.debugSamplesToConsole(exportHRBFSamplesStatus);
	}

	// print HRBF values if requested
	if (exportHRBFValuesStatusNow != exportHRBFValuesStatus) {
		std::cout << "instructed to export HRBF values: " << exportHRBFValuesStatusNow.c_str() << std::endl;
		exportHRBFValuesStatus = exportHRBFValuesStatusNow;
		// TODO: handle exporting HRBFs to the text file format
		hrbfMan.debugValuesToConsole(exportHRBFValuesStatus);
	}

	// print HRBF composition if requested
	if (exportCompositionStatusNow != exportCompositionStatus) {
		std::cout << "instructed to export HRBF composition." << std::endl;
		exportCompositionStatus = exportCompositionStatusNow;
		// TODO: handle exporting HRBFs to the text file format
		hrbfMan.debugCompositionToConsole(boneTFs, numTransforms);
	}

	// check the HRBF value if the new point is significantly different
	MPoint checkHRBFHereNow(data[0], data[1], data[2]);
	if ((checkHRBFHereNow - checkHRBFHere).length() > 0.0001) {
		if (hrbfMan.m_HRBFs.size() == numTransforms) {
			std::cout << "checking HRBF at x:" << data[0] << " y: " << data[1] << " z: " << data[2] << std::endl;
			hrbfMan.compose(boneTFs);
			float val = 0.0f;
			float dx = 0.0f;
			float dy = 0.0f;
			float dz = 0.0f;
			float grad = 0.0f;

			hrbfMan.mf_vals->trilinear(data[0], data[1], data[2], val);
			hrbfMan.mf_gradX->trilinear(data[0], data[1], data[2], dx);
			hrbfMan.mf_gradY->trilinear(data[0], data[1], data[2], dy);
			hrbfMan.mf_gradZ->trilinear(data[0], data[1], data[2], dz);
			hrbfMan.mf_gradMag->trilinear(data[0], data[1], data[2], grad);
			std::cout << "val: " << val << " dx: " << dx << " dy: " << dy << " dz: " << dz << " grad: " << grad << std::endl;
			checkHRBFHere = checkHRBFHereNow;
		}
	}

	// rebuild HRBFs if needed
	if (signalRebuildHRBF) {
		std::cout << "instructed to rebuild HRBFs" << std::endl;
		rebuildHRBFStatus = rebuildHRBFStatusNow;

		MArrayDataHandle parentIDCsHandle = block.inputArrayValue(jointParentIdcs); // tell block what we want
		std::vector<int> jointParentIndices(numTransforms);
		if (parentIDCsHandle.elementCount() > 0) {
			for (int i = 0; i<numTransforms; ++i) {
				jointParentIndices[i] = parentIDCsHandle.inputValue().asInt();
				parentIDCsHandle.next();
			}
		}

		MArrayDataHandle jointNamesHandle = block.inputArrayValue(jointNames); // tell block what we want
		std::vector<std::string> jointNames(numTransforms);
		if (jointNamesHandle.elementCount() > 0) {
			for (int i = 0; i<numTransforms; ++i) {
				jointNames[i] = jointNamesHandle.inputValue().asString().asChar();
				jointNamesHandle.next();
			}
		}

		// debug
		//std::cout << "got joint hierarchy info! it's:" << std::endl;
		//for (int i = 0; i < numTransforms; ++i) {
		//	std::cout << i << ": " << jointNames[i].c_str() << " : " << jointParentIndices[i] << std::endl;
		//}
		std::cout << "rebuilding HRBFs... " << std::endl;
		hrbfMan.buildHRBFs(jointParentIndices, jointNames, bindTFs, boneTFs, 
			weightListHandle, iter, weights);
		std::cout << "done rebuilding!" << std::endl;
		weightListHandle.jumpToElement(0); // reset this, it's an iterator. trust me.
		iter.reset(); // reset this iterator so we can go do normal skinning
	}


	// perform traditional skinning
	if (useDQNow != 0) {
		returnStatus = skinDQ(transforms, numTransforms, weightListHandle, iter);
	}
	else {
		returnStatus = skinLB(transforms, numTransforms, weightListHandle, iter);
	}

	// do HRBF corrections
	if (useHRBFnow != 0) {
		if (hrbfMan.m_HRBFs.size() == numTransforms) {
			hrbfMan.compose(boneTFs);
			iter.reset();
			hrbfMan.correct(iter);
		}
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
		// get the weights for this point -> must be dependent on the iterator somehow
		MArrayDataHandle weightsHandle = weightListHandle.inputValue().child(weights);
		// compute the skinning -> TODO: what's the order that the weights are given in? Appears to just be maya list relatives order.
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
