#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include <vtkSmartPointer.h>
#include <vtkTubeFilter.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkTriangleFilter.h>
#include <vtkNew.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/geometric_shape_to_BVH_model.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>

#include <blaze/Math.h>

class collisionCheck
{

public:
    // default constructor
    collisionCheck();

    // collisionCheck constructor
    collisionCheck(double tubeRadius);

    // collisionCheck desctructor
    ~collisionCheck() = default;

    // copy constructor
    collisionCheck(const collisionCheck &rhs);

    // move constructor
    collisionCheck(collisionCheck &&rhs) noexcept;

    // Copy assignment operator
    collisionCheck &operator=(const collisionCheck &rhs);

    // move assignment operator
    collisionCheck &operator=(collisionCheck &&rhs) noexcept;

    // function that constructs a fcl::BVHModel for the CTR from its discretized backbone points
    std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> generateBVHModel(const blaze::HybridMatrix<double, 3UL, 1000UL, blaze::rowMajor> &POINTS);

    // function that computes the minimum distance between the CTR and anatomy
    double computeMinimumDistance();

    // function that computes the number of collisions between the CTR and anatomy
    size_t computeCollisions();
    
    // function that sets the shared pointer to the skeleton BVHModel
    void setSkeletonBVHModel(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> &skeletonMesh);

    // function that sets the shared pointer to the liver BVHModel
    void setLiverBVHModel(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> &liverMesh);

    // function that sets the shared pointer to the spleen BVHModel
    void setSpleenBVHModel(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> &spleenMesh);

    // function that sets the shared pointer to the lung BVHModel
    void setLungBVHModel(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> &lungMesh);

    // function that updates all anatomy mesh references after any change in the pointers
    void updateAnatomyMeshReferences();

    // function that updates the CTR mesh references after any change in the pointers
    void updateCTRMeshReferences();

    //function that sets the backbone points
    void setBackbonePoints(const blaze::HybridMatrix<double, 3UL, 1000UL, blaze::rowMajor> &Points);

    // returns the shared pointer to the ctr mesh
    std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> getCTR_Mesh();

private:
    double m_tubeRadius;
    std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> m_skeletonMesh, m_liverMesh, m_spleenMesh, m_lungMesh, m_ctrMesh;
    std::shared_ptr<fcl::CollisionObjectd> m_skeletonObj, m_liverObj, m_spleenObj, m_lungObj, m_ctrObj;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> m_anatomyManager, m_ctrManager;
    std::vector<std::shared_ptr<fcl::CollisionObjectd>> m_anatomyObjects, m_ctrObject;
    fcl::DefaultCollisionData<double> m_collisionData;
	fcl::DefaultDistanceData<double> m_distanceData;    
};