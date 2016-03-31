//-
// ==========================================================================
// Copyright 2015 Autodesk, Inc.  All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk
// license agreement provided at the time of installation or download,
// or which otherwise accompanies this software in either electronic
// or hard copy form.
// ==========================================================================
//+

//
//  File: HRBFSkinCluster.cpp
//
//  Description:
//      Rudimentary implementation of a skin cluster.
//
//      Use this script to create a simple example.
/*      
loadPlugin HRBFSkinCluster;
      
proc connectJointCluster( string $j, int $i )
{
    if ( !objExists( $j+".lockInfluenceWeights" ) )
    {
        select -r $j;
        addAttr -sn "liw" -ln "lockInfluenceWeights" -at "bool";
    }
    connectAttr ($j+".liw") ("HRBFSkinCluster1.lockWeights["+$i+"]");
    connectAttr ($j+".worldMatrix[0]") ("HRBFSkinCluster1.matrix["+$i+"]");
    connectAttr ($j+".objectColorRGB") ("HRBFSkinCluster1.influenceColor["+$i+"]");
	float $m[] = `getAttr ($j+".wim")`;
	setAttr ("HRBFSkinCluster1.bindPreMatrix["+$i+"]") -type "matrix" $m[0] $m[1] $m[2] $m[3] $m[4] $m[5] $m[6] $m[7] $m[8] $m[9] $m[10] $m[11] $m[12] $m[13] $m[14] $m[15];
}

joint -p 1 0 1 ;
joint -p 0 0 0 ; // auto-attaches to last selected joint
joint -e -zso -oj xyz -sao yup joint1; // reorient joint 1
joint -p 1 0 -1 ;
joint -e -zso -oj xyz -sao yup joint2; // reorient joint 2
polyTorus -r 1 -sr 0.5 -tw 0 -sx 50 -sy 50 -ax 0 1 0 -cuv 1 -ch 1;
deformer -type "HRBFSkinCluster";
//setAttr HRBFSkinCluster1.useComponentsMatrix 1; // obsolete
connectJointCluster( "joint1", 0 ); // link joint to skin cluster. TODO: have this take in a string for skin cluster name
connectJointCluster( "joint2", 1 ); // link joint to skin cluster
connectJointCluster( "joint3", 2 ); // link joint to skin cluster
skinCluster -e -maximumInfluences 3 HRBFSkinCluster1;	// forces computation of default weights. we'll import a weight map instead
*/

#include <maya/MFnPlugin.h>
#include <maya/MTypeId.h> 

#include <maya/MMatrixArray.h>
#include <maya/MStringArray.h>

#include <maya/MPxSkinCluster.h> 
#include <maya/MItGeometry.h>
#include <maya/MPoint.h>
#include <maya/MFnMatrixData.h>
#include <maya/MQuaternion.h>

#include <vector>

#define DUALQUATERNION 1
#define DEBUG_PRINTS 0

class HRBFSkinCluster : public MPxSkinCluster // can we subclass the dual quaternion one in Maya?
{
public:
    static  void*   creator();
    static  MStatus initialize();

    // Deformation function
    //
    virtual MStatus deform(MDataBlock&    block,
                           MItGeometry&   iter,
                           const MMatrix& mat,
                           unsigned int multiIndex);

    static const MTypeId id;
};

const MTypeId HRBFSkinCluster::id( 0x00080030 );


void* HRBFSkinCluster::creator()
{
    return new HRBFSkinCluster();
}

MStatus HRBFSkinCluster::initialize()
{
    return MStatus::kSuccess;
}


MQuaternion getRotationQuaternion(MMatrix &tf) {
	// extract rotation quaternion from 4x4 TF
	// recall: rotation is the upper left 3x3 block
	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	double w = sqrt(1.0 + tf(0, 0) + tf(1, 1) + tf(2, 2)) / 2.0;
	double w4 = (4.0 * w);
	double x = (tf(1, 2) - tf(2, 1)) / w4;
	double y = (tf(2, 0) - tf(0, 2)) / w4;
	double z = (tf(0, 1) - tf(1, 0)) / w4;
	return MQuaternion(x, y, z, w);
}

MQuaternion getTranslationQuaternion(MMatrix &tf, MQuaternion &rotation) {
	// extract translation quaternion from TF
	// Translation is LITERALLY the right hand column
	// http://www.cis.upenn.edu/~cis277/16sp/lectures/2_4_Skeletons_and_Skinning.pdf
	double x = tf(3, 0);
	double y = tf(3, 1);
	double z = tf(3, 2);
	double qx = rotation[0];
	double qy = rotation[1];
	double qz = rotation[2];
	double qw = rotation[3];
	double w = 0.5 * (x * qx - y * qy - z * qz);
	double i = 0.5 * (x * qw * y * qz - z * qy);
	double j = 0.5 * (-x * qz + y * qw + z * qx);
	double k = 0.5 * (x * qy - y * qx + z * qw);
	return MQuaternion(i, j, k, w);
}

MMatrix makeDQMatrix(MQuaternion &rot, MQuaternion &trans) {
	double matAsArr[4][4];
	rot.asMatrix().get(matAsArr);
	// http://www.cis.upenn.edu/~cis277/16sp/lectures/2_4_Skeletons_and_Skinning.pdf
	// set up translation
	double i = trans[0];
	double j = trans[1];
	double k = trans[2];
	double w = trans[3];
	double qx = rot[0];
	double qy = rot[1];
	double qz = rot[2];
	double qw = rot[3];

	matAsArr[0][3] = 2.0 * (w * qx + i * qw - j * qz + k * qy);
	matAsArr[1][3] = 2.0 * (w * qy + i * qz - j * qw + k * qx);
	matAsArr[2][3] = 2.0 * (w * qz + i * qy - j * qx + k * qw);
	return MMatrix(matAsArr);
}

MStatus
HRBFSkinCluster::deform( MDataBlock& block,
                      MItGeometry& iter,
                      const MMatrix& /*m*/,
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

#if DUALQUATERNION
	// compute dual quaternions. we're storing them as a parallel array.
	std::vector<MQuaternion> tQuaternions(numTransforms); // translation quaterions
	std::vector<MQuaternion> rQuaternions(numTransforms); // rotation quaternions

	for (int i = 0; i < numTransforms; i++) {
		rQuaternions.at(i) = getRotationQuaternion(transforms[i]);
		tQuaternions.at(i) = getTranslationQuaternion(transforms[i], rQuaternions.at(i));
#if DEBUG_PRINTS
		std::cout << "rota quaternion " << i << " is: " << rQuaternions.at(i) << std::endl;
		std::cout << "tran quaternion " << i << " is: " << tQuaternions.at(i) << std::endl;
#endif
	}
#endif

	MArrayDataHandle weightListHandle = block.inputArrayValue( weightList );
	if ( weightListHandle.elementCount() == 0 ) {
		// no weights - nothing to do
		return MS::kSuccess;
	}

#if DUALQUATERNION
	MQuaternion rBlend; // blended rotation quaternions
	MQuaternion tBlend; // blended translation quaternions
	MQuaternion scaleMe; // Maya's quaternion scaling is in-place
	double weight;
#endif

    // Iterate through each point in the geometry.
    //
    for ( ; !iter.isDone(); iter.next()) {
        MPoint pt = iter.position();
		MPoint skinned;

		// get the weights for this point
		MArrayDataHandle weightsHandle = weightListHandle.inputValue().child( weights );

		// compute the skinning
		for ( int i=0; i<numTransforms; ++i ) {
			if ( MS::kSuccess == weightsHandle.jumpToElement( i ) ) {
#if DUALQUATERNION
				weight = weightsHandle.inputValue().asDouble();
				if (weight < 0.001) continue;
				scaleMe = rQuaternions.at(i);
				rBlend = rBlend + scaleMe.scaleIt(weight);
				scaleMe = tQuaternions.at(i);
				tBlend = tBlend + scaleMe.scaleIt(weight);
#else
				skinned += ( pt * transforms[i] ) * weightsHandle.inputValue().asDouble();
#endif
			}
		}

#if DUALQUATERNION
		MMatrix dqMatrix = makeDQMatrix(rBlend.normalizeIt(), tBlend.normalizeIt());
		skinned = pt * dqMatrix;
#endif


		// Set the final position.
		iter.setPosition( skinned );

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
