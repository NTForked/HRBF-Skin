#ifndef MAYADQUTILS_H
#define MAYADQUTILS_H

#include <maya/MFnMatrixData.h>
#include <maya/MMatrix.h>
#include <maya/MQuaternion.h>

MQuaternion getRotationQuaternion(MMatrix &tf);

MQuaternion getTranslationQuaternion(MMatrix &tf, MQuaternion &rotation);

MMatrix makeDQMatrix(MQuaternion &rot, MQuaternion &trans);

#endif