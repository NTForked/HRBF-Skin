#ifndef MAYAHRBF_H
#define MAYAHRBF_H

#include <maya/MVector.h>
#include <maya/MMatrix.h>
#include "FloatGrid3D.hpp"
#include "hrbf3.hpp"
#include <vector>
#include <algorithm>

#define HRBF_RES 32
#define CULL_H 0.05f
#define SAMPLE_CAP 60

class MayaHRBF {
public:
	MayaHRBF(std::string &name, MMatrix &invBindTF);
	~MayaHRBF();

	void setupBones();
	void addVertex(MPoint pos, MVector nor); // take vertex into local space. handle culling.
	void compute();

	void query(float x, float y, float z, // expecting local coordinates
		float &val, float &gradX, float &gradY, float &gradZ, float &gradM);
	void query(MPoint pos, float &val, float &gradX, float &gradY, float &gradZ, float &gradM);

	void printHRBFSamplingDebug();
	void printHRBF(); // TODO: implement!

	std::string m_name;
	MayaHRBF* m_parent;
	std::vector<MayaHRBF*> m_children;

	void reduceSamples();
	bool isExtremity();
	void closeExtremities();

	// members
	int m_numChildren;

	MMatrix m_invBindTF;
	MMatrix m_bindTF;

	// samples from geometry
	std::vector<MPoint> m_posSamples;
	std::vector<MVector> m_norSamples;

	// extremities
	std::vector<MPoint> m_posExtrem;
	std::vector<MVector> m_norExtrem;

	// cache of bone positions for sample culling
	std::vector<MPoint> m_jointPosLocals; // position of joint in local space
	std::vector<MVector> m_jointDirLocals; // direction from this joint to given joint
	MPoint m_jointPositionLocal; // this joint in local space
	MPoint m_jointPositionWorld; // this joint in world space

	// HRBF and reparameterization stuff
	float m_r; // for compact support


	FloatGrid3D *mf_vals;
	FloatGrid3D *mf_gradX;
	FloatGrid3D *mf_gradY;
	FloatGrid3D *mf_gradZ;
	FloatGrid3D *mf_gradMag;
};

#endif