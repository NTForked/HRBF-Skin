#ifndef HRBF3_H
#define HRBF3_H

// based heavily off Rodolphe Vaillant and Gael Guennebaud's hrbf_core
// rewritten to talk to Maya's API more easily
// gael.guennebaud@inria.fr - http://www.labri.fr/perso/guenneba/
// Rodolphe Vaillant - (Fixed the gradient evaluation) - http://www.irit.fr/~Rodolphe.Vaillant

#include <Eigen\LU>
#include <Eigen\Cholesky>
#include <vector>
#include <maya/MVector.h>

class HRBF3
{
	// define some types based on Eigen's templates.
	typedef Eigen::Matrix<double, 3, 3> mat3;
	typedef Eigen::Matrix<double, 3, 1> vec3;
	typedef Eigen::Matrix<double, 3, Eigen::Dynamic> mat3X;
	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matXX;
	typedef Eigen::Matrix<double, Eigen::Dynamic, 1> vecX;

public:
	HRBF3(std::vector<MVector> &points, std::vector<MVector> &normals); // build the HRBF
	~HRBF3();

	float evaluate(MVector pos);
	float evaluate(float x, float y, float z);

	MVector gradient(MVector pos);
	void gradient(float x, float y, float z, float &dx, float &dy, float &dz);

private:
	// members
	mat3X m_node_centers; // columns representing p_i from the input
	vecX m_alphas; // alphas
	mat3X m_betas; // columns representing betas
};

#endif