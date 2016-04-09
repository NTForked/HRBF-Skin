#include "MayaDualQuaternion.h"

MQuaternion getRotationQuaternion(MMatrix &tf) {
	// extract rotation quaternion from 4x4 TF
	// recall: rotation is the upper left 3x3 block
	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	// the matrix implementation in the example appears to be transposed relative to Maya's
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

	double t[3];
	t[0] = tf(3, 0);
	t[1] = tf(3, 1);
	t[2] = tf(3, 2);
	double q0[4];
	q0[0] = rotation[3];
	q0[1] = rotation[0];
	q0[2] = rotation[1];
	q0[3] = rotation[2];

	double w = -0.5*(t[0] * q0[1] + t[1] * q0[2] + t[2] * q0[3]);
	double i = 0.5*(t[0] * q0[0] + t[1] * q0[3] - t[2] * q0[2]);
	double j = 0.5*(-t[0] * q0[3] + t[1] * q0[0] + t[2] * q0[1]);
	double k = 0.5*(t[0] * q0[2] - t[1] * q0[1] + t[2] * q0[0]);

	return MQuaternion(i, j, k, w);
}

MMatrix makeDQMatrix(MQuaternion &rot, MQuaternion &trans) {
	double matAsArr[4][4];
	rot.asMatrix().get(matAsArr);

	double length = sqrt(rot[0] * rot[0] + rot[1] * rot[1] + rot[2] * rot[2] + rot[3] * rot[3]);

	double t[4];
	double q0[4];
	t[0] = trans[3];
	t[1] = trans[0];
	t[2] = trans[1];
	t[3] = trans[2];

	q0[0] = rot[3];
	q0[1] = rot[0];
	q0[2] = rot[1];
	q0[3] = rot[2];

	matAsArr[3][0] = 2.0*(-t[0] * q0[1] + t[1] * q0[0] - t[2] * q0[3] + t[3] * q0[2]) / length;
	matAsArr[3][1] = 2.0*(-t[0] * q0[2] + t[1] * q0[3] + t[2] * q0[0] - t[3] * q0[1]) / length;
	matAsArr[3][2] = 2.0*(-t[0] * q0[3] - t[1] * q0[2] + t[2] * q0[1] + t[3] * q0[0]) / length;

	return MMatrix(matAsArr);
}
