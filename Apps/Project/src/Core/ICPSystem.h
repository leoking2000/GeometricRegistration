#pragma once
#include <geo/GeometricRegistration.h>

enum class ICPMethod : geo::u8
{
    NAIVE = 0,
    NAIVE_PLANE = 1,
    SPARSE = 2
};

class ICPSystem
{
public:
    ICPSystem(ICPMethod method, geo::PointCloud3D target, geo::PointCloud3D source);
public:
    const geo::PointCloud3D& GetTarget() const;
    const geo::PointCloud3D& GetSource() const;
    void SetTarget(geo::PointCloud3D target);
    void SetSource(geo::PointCloud3D source);
public:
    geo::ICPResult Solve(int max_iterations = 100);
    geo::ICPResult Step();
    float GetRMS() const;
public:
    void SetSolveMethod(ICPMethod method);
public:
    static void PrintCloudPreview(const geo::PointCloud3D& cloud);
    static void PrintRigidTransform(const geo::RigidTransform& tf);
private:
    geo::PointCloud3D m_target;
    geo::PointCloud3D m_source;
    geo::KDTree m_tree;
    ICPMethod m_method = ICPMethod::NAIVE;
    float m_RMS = -1.0f;
};



