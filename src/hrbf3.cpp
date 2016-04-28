#include "hrbf3.hpp"

HRBF3::HRBF3(std::vector<MVector> &points, std::vector<MVector> &normals) {
	int numPoints = points.size();
	int numConstraints = 4 * numPoints; // num constraints is based on num normals
	int numCoeffs = numConstraints;

	m_node_centers.resize(3, numPoints);
	m_betas.resize(3, numPoints); // betas are vectors
	m_alphas.resize(numPoints); // alphas are scalars

	// allocate size for matrix to be solved and the set of constraints and coefficients
	matXX D(numConstraints, numCoeffs);
	vecX f(numConstraints);
	vecX x(numCoeffs);

	// copy node centers (sample points)
	for (int i = 0; i < numPoints; i++) {
		m_node_centers.col(i)[0] = points[i].x;
		m_node_centers.col(i)[1] = points[i].y;
		m_node_centers.col(i)[2] = points[i].z;
	}

	// assemble the D matrix
	vec3 p;
	vec3 n;
	int io; // indexer into the matrix
	int jo; // indexer into the matrix
	vec3 diff;
	double l, w, dw_l, ddw;
	vec3 g;

	for (int i = 0; i < numPoints; i++) {
		p[0] = points[i].x;
		p[1] = points[i].y;
		p[2] = points[i].z;
		n[0] = normals[i].x;
		n[1] = normals[i].y;
		n[2] = normals[i].z;

		io = 4 * i; // indexer into the matrix
		f(io) = 0;
		f.template segment<3>(io + 1) = n; // add normal constraint in system

		for (int j = 0; j < numPoints; j++) {
			jo = 4 * j; // indexer into the matrix
			diff = p - m_node_centers.col(j);
			l = diff.norm();
			if (l == 0) {
				D.template block<4, 4>(io, jo).setZero();
			}
			else {
				w = l * l * l; // phi = x^3
				dw_l = 3.0 * l; // d_phi(l) / l
				ddw = 6.0 * l; // dd_phi = 6x
				g = diff * dw_l;
				// see vaillant HRBF supplement: http://rodolphe-vaillant.fr/?e=12
				D(io, jo) = w;
				D.row(io).template segment<3>(jo + 1) = g.transpose();
				D.col(jo).template segment<3>(io + 1) = g;
				D.template block<3, 3>(io + 1, jo + 1) = (ddw - dw_l) / (l*l) * (diff * diff.transpose());
				D.template block<3, 3>(io + 1, jo + 1).diagonal().array() += dw_l;
			}
		}

	}
	x = D.lu().solve(f);
	Eigen::Map< Eigen::Matrix<double, 4, Eigen::Dynamic> > mx(x.data(), 4, numPoints);

	m_alphas = mx.row(0);
	m_betas = mx.template bottomRows<3>();
}

HRBF3::~HRBF3() {

}

float HRBF3::evaluate(MVector pos) {
	float ret = 0;
	vec3 p;
	p[0] = pos.x;
	p[1] = pos.y;
	p[2] = pos.z;
	int numNodes = m_node_centers.cols();
	vec3 diff;
	double l;

	for (int i = 0; i < numNodes; ++i)
	{
		diff = p - m_node_centers.col(i);
		l = diff.norm();

		if (l > 0)
		{
			ret += m_alphas(i) * l * l * l; // phi = x^3
			ret += m_betas.col(i).dot(diff) * 3.0 * l; // d_phi(l) / l = 3x^2 / x = 3x
		}
	}

	return ret;
}

float HRBF3::evaluate(float x, float y, float z) {
	float ret = 0;
	vec3 p;
	p[0] = x;
	p[1] = y;
	p[2] = z;
	int numNodes = m_node_centers.cols();
	vec3 diff;
	double l;

	for (int i = 0; i < numNodes; ++i)
	{
		diff = p - m_node_centers.col(i);
		l = diff.norm();

		if (l > 0)
		{
			ret += m_alphas(i) * l * l * l; // phi = x^3
			ret += m_betas.col(i).dot(diff) * 3.0 * l; // d_phi(l) / l = 3x^2 / x = 3x
		}
	}

	return ret;
}

MVector HRBF3::gradient(MVector pos) {
	vec3 grad = vec3::Zero();
	vec3 p;
	p[0] = pos.x;
	p[1] = pos.y;
	p[2] = pos.z;

	vec3 node, beta, diff, diffNormalized;
	double alpha, l, dphi, ddphi, alpha_dphi, bDotd_l, squared_l;

	int nb_nodes = m_node_centers.cols();
	for (int i = 0; i < nb_nodes; i++)
	{
		node = m_node_centers.col(i);
		beta = m_betas.col(i);
		alpha = m_alphas(i);
		diff = p - node;

		diffNormalized = diff;
		l = diff.norm();

		if (l > 0.00001)
		{
			diffNormalized.normalize();
			dphi = 3.0 * l * l;
			ddphi = 6.0 * l;

			alpha_dphi = alpha * dphi;

			bDotd_l = beta.dot(diff) / l;
			squared_l = diff.squaredNorm();

			grad += alpha_dphi * diffNormalized;
			grad += bDotd_l * (ddphi * diffNormalized - diff * dphi / squared_l) + beta * dphi / l;
		}
	}
	return MVector(grad[0], grad[1], grad[2]);
}

void HRBF3::gradient(float x, float y, float z, float &dx, float &dy, float &dz) {
	vec3 grad = vec3::Zero();
	vec3 p;
	p[0] = x;
	p[1] = y;
	p[2] = z;

	vec3 node, beta, diff, diffNormalized;
	double alpha, l, dphi, ddphi, alpha_dphi, bDotd_l, squared_l;

	int nb_nodes = m_node_centers.cols();
	for (int i = 0; i < nb_nodes; i++)
	{
		node = m_node_centers.col(i);
		beta = m_betas.col(i);
		alpha = m_alphas(i);
		diff = p - node;

		diffNormalized = diff;
		l = diff.norm();

		if (l > 0.00001)
		{
			diffNormalized.normalize();
			dphi = 3.0 * l * l;
			ddphi = 6.0 * l;

			alpha_dphi = alpha * dphi;

			bDotd_l = beta.dot(diff) / l;
			squared_l = diff.squaredNorm();

			grad += alpha_dphi * diffNormalized;
			grad += bDotd_l * (ddphi * diffNormalized - diff * dphi / squared_l) + beta * dphi / l;
		}
	}
	dx = grad[0];
	dy = grad[1];
	dz = grad[2];
}