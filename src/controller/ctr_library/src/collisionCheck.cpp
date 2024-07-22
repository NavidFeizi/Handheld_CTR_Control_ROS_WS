// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: http://www.viva64.com
#include "collisionCheck.hpp"

// default constructor
collisionCheck::collisionCheck()
{
    m_tubeRadius = 1.00;
    // mesh BVHModel objects
    m_skeletonMesh = nullptr;
    m_liverMesh = nullptr;
    m_spleenMesh = nullptr;
    m_lungMesh = nullptr;
    m_ctrMesh = nullptr;
    // collision objects
    m_skeletonObj = nullptr;
    m_liverObj = nullptr;
    m_spleenObj = nullptr;
    m_lungObj = nullptr;
    m_ctrObj = nullptr;
    // collision managers
    m_anatomyManager = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
    m_ctrManager = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
}

// collisionCheck overloaded constructor
collisionCheck::collisionCheck(double tubeRadius) : m_tubeRadius(tubeRadius)
{
    // mesh BVHModel objects
    m_skeletonMesh = nullptr;
    m_liverMesh = nullptr;
    m_spleenMesh = nullptr;
    m_lungMesh = nullptr;
    m_ctrMesh = nullptr;
    // collision objects
    m_skeletonObj = nullptr;
    m_liverObj = nullptr;
    m_spleenObj = nullptr;
    m_lungObj = nullptr;
    m_ctrObj = nullptr;
    // collision managers
    m_anatomyManager = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
    m_ctrManager = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();

    // updates the BVHModel references for the new meshes
    this->updateAnatomyMeshReferences();
}

// copy constructor
collisionCheck::collisionCheck(const collisionCheck &rhs) : m_tubeRadius(rhs.m_tubeRadius), m_skeletonMesh(rhs.m_skeletonMesh), m_liverMesh(rhs.m_liverMesh), m_spleenMesh(rhs.m_spleenMesh), m_lungMesh(rhs.m_lungMesh), m_skeletonObj(rhs.m_skeletonObj), m_liverObj(rhs.m_liverObj), m_spleenObj(rhs.m_spleenObj), m_lungObj(rhs.m_lungObj), m_anatomyObjects(rhs.m_anatomyObjects)
{
    // ctr mesh & collision object does not get copied - must be independet for each CTR object
    this->m_ctrObject.clear();
    this->m_ctrMesh = nullptr;
    this->m_ctrObj = nullptr;
    this->m_anatomyManager = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
    this->m_ctrManager = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();

    // updates the BVHModel references for the new meshes
    this->updateAnatomyMeshReferences();
}

// move constructor
collisionCheck::collisionCheck(collisionCheck &&rhs) noexcept
{
    // handling self assignment
    if (this != &rhs)
    {
        m_tubeRadius = rhs.m_tubeRadius;
        // organs & ctr meshes
        m_skeletonMesh = std::move(rhs.m_skeletonMesh);
        m_liverMesh = std::move(rhs.m_liverMesh);
        m_spleenMesh = std::move(rhs.m_spleenMesh);
        m_lungMesh = std::move(rhs.m_lungMesh);
        m_ctrMesh = std::move(rhs.m_ctrMesh);
        // collision objects
        m_skeletonObj = std::move(rhs.m_skeletonObj);
        m_liverObj = std::move(rhs.m_liverObj);
        m_spleenObj = std::move(rhs.m_spleenObj);
        m_lungObj = std::move(rhs.m_lungObj);
        m_ctrObj = std::move(rhs.m_ctrObj);
        // collision managers
        m_anatomyManager = std::move(rhs.m_anatomyManager);
        m_ctrManager = std::move(rhs.m_ctrManager);
        // vectors of collision objects
        m_anatomyObjects = std::move(rhs.m_anatomyObjects);
        m_ctrObject = std::move(rhs.m_ctrObject);
    }
}

// Copy assignment operator
collisionCheck &collisionCheck::operator=(const collisionCheck &rhs)
{
    // handling self assignment
    if (this != &rhs)
    {
        m_tubeRadius = rhs.m_tubeRadius;
        // organs & ctr meshes
        m_skeletonMesh = rhs.m_skeletonMesh;
        m_liverMesh = rhs.m_liverMesh;
        m_spleenMesh = rhs.m_spleenMesh;
        m_lungMesh = rhs.m_lungMesh;
        m_ctrMesh = nullptr; // rhs.m_ctrMesh;
        // collision objects
        m_skeletonObj = rhs.m_skeletonObj;
        m_liverObj = rhs.m_liverObj;
        m_spleenObj = rhs.m_spleenObj;
        m_lungObj = rhs.m_lungObj;
        m_ctrObj = nullptr; // std::make_shared<fcl::CollisionObjectd>(m_ctrMesh);
        // collision managers
        m_anatomyManager = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
        m_ctrManager = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
        // vectors of collision objects
        m_anatomyObjects = rhs.m_anatomyObjects;
        // m_ctrObject.push_back(m_ctrObj);
    }

    return *this;
}

// move assignment operator
collisionCheck &collisionCheck::operator=(collisionCheck &&rhs) noexcept
{
    // handling self assignment
    if (this != &rhs)
    {
        m_tubeRadius = rhs.m_tubeRadius;
        // organs & ctr meshes
        m_skeletonMesh = std::move(rhs.m_skeletonMesh);
        m_liverMesh = std::move(rhs.m_liverMesh);
        m_spleenMesh = std::move(rhs.m_spleenMesh);
        m_lungMesh = std::move(rhs.m_lungMesh);
        m_ctrMesh = std::move(rhs.m_ctrMesh);
        // collision objects
        m_skeletonObj = std::move(rhs.m_skeletonObj);
        m_liverObj = std::move(rhs.m_liverObj);
        m_spleenObj = std::move(rhs.m_spleenObj);
        m_lungObj = std::move(rhs.m_lungObj);
        m_ctrObj = std::move(rhs.m_ctrObj);
        // collision managers
        m_anatomyManager = std::move(rhs.m_anatomyManager);
        m_ctrManager = std::move(rhs.m_ctrManager);
        // vectors of collision objects
        m_anatomyObjects = std::move(rhs.m_anatomyObjects);
        m_ctrObject = std::move(rhs.m_ctrObject);
    }

    return *this;
}

// function that constructs a fcl::BVHModel for the CTR from its discretized backbone points
std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> collisionCheck::generateBVHModel(const blaze::HybridMatrix<double, 3UL, 1000UL, blaze::rowMajor> &POINTS)
{
    const size_t numPoints = POINTS.columns();
    static constexpr size_t numSides = 12UL;

    // Create VTK points object
    vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
    for (size_t i = 0UL; i < numPoints; ++i)
        Points->InsertNextPoint(-POINTS(0UL, i), -POINTS(1UL, i), POINTS(2UL, i));

    // Create a cell array to store the polyline
    vtkSmartPointer<vtkCellArray> CellArray = vtkSmartPointer<vtkCellArray>::New();
    CellArray->InsertNextCell(numPoints);
    for (size_t i = 0UL; i < numPoints; ++i)
        CellArray->InsertCellPoint(i);

    // Create a polydata object to hold the polyline
    vtkSmartPointer<vtkPolyData> PolyData = vtkSmartPointer<vtkPolyData>::New();
    PolyData->SetPoints(Points);
    PolyData->SetLines(CellArray);

    // Use vtkTubeFilter to create a tube around the polyline
    vtkSmartPointer<vtkTubeFilter> TubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
    TubeFilter->SetInputData(PolyData);
    TubeFilter->SetNumberOfSides(numSides); // use 16-sided polygon to approximate circular cross section
    TubeFilter->SetRadius(this->m_tubeRadius);
    TubeFilter->SetCapping(true); // cap the ends of the tube
    TubeFilter->Update();

    // Clean the polydata (removes any duplicate points and merges any coincident points)
    vtkSmartPointer<vtkCleanPolyData> CleanPolyData = vtkSmartPointer<vtkCleanPolyData>::New();
    CleanPolyData->SetInputConnection(TubeFilter->GetOutputPort());
    CleanPolyData->Update();

    // Use vtkTriangleFilter to convert the output mesh to a mesh of only triangles
    vtkSmartPointer<vtkTriangleFilter> TriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    TriangleFilter->SetInputData(CleanPolyData->GetOutput());
    TriangleFilter->Update();

     // Get the points from the polydata
    vtkPoints *meshPoints = TriangleFilter->GetOutput()->GetPoints();
    // Get the cells (triangles) from the polydata
    vtkCellArray *meshCells = TriangleFilter->GetOutput()->GetPolys();

    // Iterate over the cells and add them to the mesh
    vtkIdType npts;
    const vtkIdType *pts;
    fcl::Vector3d v1, v2, v3;

    // instantiating the BVHModel
    std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> model = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();

    // Adding mesh triangles to the model
    model->beginModel();

    for (meshCells->InitTraversal(); meshCells->GetNextCell(npts, pts);)
    {
        // Create Vector3d objects from the vtkPoints using the Eigen::Map constructor
        v1 = Eigen::Map<Eigen::Vector3d>(meshPoints->GetPoint(pts[0]));
        v2 = Eigen::Map<Eigen::Vector3d>(meshPoints->GetPoint(pts[1]));
        v3 = Eigen::Map<Eigen::Vector3d>(meshPoints->GetPoint(pts[2]));

        // Add the triangle to the mesh
        model->addTriangle(v1, v2, v3);
    }

    // finished constructing the BVHModel
    model->endModel();

    return model;
}

// function that computes the minimum distance between the CTR and anatomy
double collisionCheck::computeMinimumDistance()
{
    m_ctrManager->distance(m_anatomyManager.get(), &m_distanceData, fcl::DefaultDistanceFunction);
    // retrieving the minimum distance between the CTR and the anatomy
    const double min_Distance = m_distanceData.result.min_distance;

    return min_Distance;
}

// function that computes the number of collisions between the CTR and anatomy
size_t collisionCheck::computeCollisions()
{
    m_ctrManager->collide(m_anatomyManager.get(), &m_collisionData, fcl::DefaultCollisionFunction);
    // retrieving the number of collisions between the CTR and the anatomy
    const size_t num_Contacts = m_collisionData.result.numContacts();

    return num_Contacts;
}

// function that sets the shared pointer to the skeleton BVHModel
void collisionCheck::setSkeletonBVHModel(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> &skeletonMesh)
{
    if (this->m_skeletonMesh)
        this->m_skeletonMesh.reset();    // Release old pointer
    this->m_skeletonMesh = skeletonMesh; // Assign new pointer
    this->m_skeletonMesh->computeLocalAABB();
}

// function that sets the shared pointer to the liver BVHModel
void collisionCheck::setLiverBVHModel(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> &liverMesh)
{
    if (this->m_liverMesh)
        this->m_liverMesh.reset(); // Release old pointer
    this->m_liverMesh = liverMesh; // Assign new pointer
    this->m_liverMesh->computeLocalAABB();
}

// function that sets the shared pointer to the spleen BVHModel
void collisionCheck::setSpleenBVHModel(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> &spleenMesh)
{
    if (this->m_spleenMesh)
        this->m_spleenMesh.reset();  // Release old pointer
    this->m_spleenMesh = spleenMesh; // Assign new pointer
    this->m_spleenMesh->computeLocalAABB();
}

// function that sets the shared pointer to the lung BVHModel
void collisionCheck::setLungBVHModel(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> &lungMesh)
{
    if (this->m_lungMesh)
        this->m_lungMesh.reset(); // Release old pointer
    this->m_lungMesh = lungMesh;  // Assign new pointer
    this->m_lungMesh->computeLocalAABB();
}

// function that updates all mesh references after any change in the pointers
void collisionCheck::updateAnatomyMeshReferences()
{
    // releases all pointers in the vector of collision objects
    for (auto &obj : m_anatomyObjects)
    {
        // unregisters the old anatomy collision object from the Anatomy collision manager
        m_anatomyManager->unregisterObject(obj.get());
        // releases the smart pointer to the anatomy collision object
        obj.reset();
    }

    // clears the anatomy mesh manager
    this->m_anatomyManager->clear();

    // Now it's safe to clear up the vector of anatomical collision objects
    m_anatomyObjects.clear();

    if (m_skeletonMesh)
    {
        m_skeletonObj.reset();
        m_skeletonObj = std::make_shared<fcl::CollisionObjectd>(m_skeletonMesh);
        m_skeletonObj->computeAABB();
        m_anatomyObjects.push_back(m_skeletonObj);
        m_anatomyManager->registerObject(m_anatomyObjects.back().get());
    }

    if (m_lungMesh)
    {
        m_lungObj.reset();
        m_lungObj = std::make_shared<fcl::CollisionObjectd>(m_lungMesh);
        m_lungObj->computeAABB();
        m_anatomyObjects.push_back(m_lungObj);
        m_anatomyManager->registerObject(m_anatomyObjects.back().get());
    }

    if (m_liverMesh)
    {
        m_liverObj.reset();
        m_liverObj = std::make_shared<fcl::CollisionObjectd>(m_liverMesh);
        m_liverObj->computeAABB();
        m_anatomyObjects.push_back(m_liverObj);
        m_anatomyManager->registerObject(m_anatomyObjects.back().get());
    }

    if (m_spleenMesh)
    {
        m_spleenObj.reset();
        m_spleenObj = std::make_shared<fcl::CollisionObjectd>(m_spleenMesh);
        m_spleenObj->computeAABB();
        m_anatomyObjects.push_back(m_spleenObj);
        m_anatomyManager->registerObject(m_anatomyObjects.back().get());
    }

    // Setup the anatomy collision/distance managers & initializing the broadphase acceleration structure according to objects input
    m_anatomyManager->setup();
}

// function that updates all mesh references after any change in the pointers
void collisionCheck::updateCTRMeshReferences()
{
    if (m_ctrMesh)
    {
        /*
             unregisters the current CTR object from the so that a new one can be taken into
             account by the CTR collision manager
        */
        for (auto &obj : m_ctrObject)
        {
            // unregisters the old CTR collision object from the CTR collision manager
            m_ctrManager->unregisterObject(obj.get());
            // releases the collision object smart pointer
            obj.reset();
        }

        // clears the CTR mesh manager
        this->m_ctrManager->clear();

        // Now it's safe to clear up the vector of CTR collision objects
        m_ctrObject.clear();

        m_ctrObj.reset();
        m_ctrObj = std::make_shared<fcl::CollisionObjectd>(m_ctrMesh);
        m_ctrObj->computeAABB();
        m_ctrObject.push_back(m_ctrObj);
        m_ctrManager->registerObject(m_ctrObject.back().get());

        // Setup the CTR collision/distance managers & initializing the broadphase acceleration structure according to objects input
        this->m_ctrManager->setup();
    }
    else
        std::cerr << "A BVHModel mesh model has not been set for the CTR!" << std::endl;
}

// function that sets the backbone points
void collisionCheck::setBackbonePoints(const blaze::HybridMatrix<double, 3UL, 1000UL, blaze::rowMajor> &Points)
{
    // releases the CTR mesh smart pointer before assigning it a new pointer to a BVHModel
    this->m_ctrMesh.reset();
    // updates the CTR BVHModel and its references in the collision/Distance manager
    this->m_ctrMesh = this->generateBVHModel(Points);
    this->m_ctrMesh->computeLocalAABB();
    this->updateCTRMeshReferences();
}

// returns the shared pointer to the ctr mesh
std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> collisionCheck::getCTR_Mesh()
{
    return this->m_ctrMesh;
}