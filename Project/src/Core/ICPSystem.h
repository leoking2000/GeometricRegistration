#pragma once
#include <geo/GeometricRegistration.h>
#include "LeoRand.h"

enum class ICPMethod : leo::u8
{
    NAIVE = 0
};

class ICPSystem
{
public:
    ICPSystem(ICPMethod method = ICPMethod::NAIVE);
    ICPSystem(ICPMethod method, geo::PointCloud3D&& target, geo::PointCloud3D&& source);
public:
    const geo::PointCloud3D& GetTarget() const;
    const geo::PointCloud3D& GetSource() const;
    void SetTarget(geo::PointCloud3D&& target);
    void SetSource(geo::PointCloud3D&& source);
public:
    void Step();
    void Solve();
    float GetRMS() const;
public:
    void SetSolveMethod(ICPMethod method);
public:
    static geo::PointCloud3D GenerateRandomCloud(leo::Random& rng, leo::u32 count, leo::f32 min, leo::f32 max);
    static void PrintCloudPreview(const geo::PointCloud3D& cloud);
    static void PrintRigidTransform(const geo::RigidTransform& tf);
private:
    geo::PointCloud3D m_target;
    geo::PointCloud3D m_source;
    ICPMethod m_method = ICPMethod::NAIVE;
    float m_RMS = -1.0f;
};



