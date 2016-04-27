#ifndef HRBFNODE_H
#define HRBFNODE_H

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
#include <maya/MFnNumericData.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnCompoundAttribute.h>

#include "MayaDualQuaternion.h"
#include "MayaHRBFManager.h"
#include "MayaHRBF.h"

#define DUALQUATERNION 1
#define DEBUG_PRINTS 0

#define McheckErr(stat,msg)			\
	if ( MS::kSuccess != stat ) {	\
		cerr << msg;				\
		return MS::kFailure;		\
			}

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

	MStatus skinDQ(MMatrixArray&  transforms,
		int numTransforms,
		MArrayDataHandle& weightListHandle,
		MItGeometry&   iter);

	MStatus skinLB(MMatrixArray&  transforms,
		int numTransforms,
		MArrayDataHandle& weightListHandle,
		MItGeometry&   iter);

	MayaHRBFManager hrbfMan;

	static const MTypeId id;

	static MObject rebuildHRBF; // for signalling that the user wants the HRBFs recomputed
	int rebuildHRBFStatus; // for the program to check if it has rebuilt HRBFs or not
	static MObject useDQ; // for switching between DQ and LBS skinning

	static MObject jointParentIdcs; // for getting skeleton hierarchy information into the node
};

const MTypeId HRBFSkinCluster::id(0x00080030);

MObject HRBFSkinCluster::rebuildHRBF;
MObject HRBFSkinCluster::useDQ;
MObject HRBFSkinCluster::jointParentIdcs;

#endif