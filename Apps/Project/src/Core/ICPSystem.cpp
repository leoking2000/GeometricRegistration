#include <iostream>
#include <sstream>
#include <iomanip>
#include <geo/GeometricRegistration.h>
#include "ICPSystem.h"

ICPSystem::ICPSystem(ICPMethod method, geo::PointCloud3D target, geo::PointCloud3D source)
	:
	m_method(method),
	m_target(std::move(target)),
	m_source(std::move(source)),
	m_tree(m_target.GetPoints())
{
}

const geo::PointCloud3D& ICPSystem::GetTarget() const
{
	return m_target;
}

const geo::PointCloud3D& ICPSystem::GetSource() const
{
	return m_source;
}

void ICPSystem::SetTarget(geo::PointCloud3D target)
{
	m_target = std::move(target);
	m_tree = geo::KDTree(m_target.GetPoints());
}

void ICPSystem::SetSource(geo::PointCloud3D source)
{
	m_source = std::move(source);
}

geo::ICPResult ICPSystem::Solve(int max_iterations)
{
	geo::ICPResult r;

	switch (m_method)
	{
	case ICPMethod::NAIVE:
		r = geo::LeastSquaresICP(m_target, m_source, m_tree, { (geo::u32)max_iterations, 1e-5f, false });
		break;
	case ICPMethod::NAIVE_PLANE:
		r = geo::LeastSquaresICP(m_target, m_source, m_tree, { (geo::u32)max_iterations, 1e-5f, true });
		break;
	case ICPMethod::SPARSE:
		geo::SparseICPParameters p = {};
		p.maxIterations = max_iterations;
		p.p = 0.4f;
		r = geo::SparseICP(m_target, m_source, m_tree, p);
		break;
	}

	m_RMS = r.rmse;

	return r;
}


geo::ICPResult ICPSystem::Step()
{
	return Solve(1);
}

float ICPSystem::GetRMS() const
{
	return m_RMS;
}

void ICPSystem::SetSolveMethod(ICPMethod method)
{
	m_method = method;
}

void ICPSystem::PrintCloudPreview(const geo::PointCloud3D& cloud)
{
	std::cout << "Number of points: " << cloud.Size() << std::endl;
	glm::vec3 center = cloud.Centroid();
	std::cout << "Centroid: " << "(" << center.x << ", " << center.y << ", " << center.z << ")\n";

	int i = 0;
	for (const auto& p : cloud)
	{
		std::cout << "(" << p.x << ", " << p.y << ", " << p.z << ")\n";
		i++;
		if (i > 10) {
			break;
		}
	}
}

void ICPSystem::PrintRigidTransform(const geo::RigidTransform& tf)
{
	std::cout << std::fixed << std::setprecision(6);

	std::cout << "Rotation (R):\n";
	for (int r = 0; r < 3; ++r)
	{
		std::cout << "  [ ";
		for (int c = 0; c < 3; ++c)
		{
			std::cout << std::setw(10) << tf.rotation[c][r] << " ";
		}
		std::cout << "]\n";
	}

	std::cout << "\nTranslation (t):\n";
	std::cout << "  [ "
		<< tf.translation.x << ", "
		<< tf.translation.y << ", "
		<< tf.translation.z << " ]\n";

	std::cout << "\nDeterminant(R): "
		<< glm::determinant(tf.rotation)
		<< "\n\n";
}
