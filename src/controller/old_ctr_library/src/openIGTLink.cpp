#include "openIGTLink.hpp"

// overloaded constructor
openIGTLink::openIGTLink(char *hostname, int port) : m_hostname(hostname), m_port(port)
{
    m_socket = igtl::ClientSocket::New();
    int status = m_socket->ConnectToServer(m_hostname, m_port);

    if (status == 0) // checking the connection status
        std::cout << "Established openIGTLink connection!" << std::endl;
    else
    {
        std::cerr << "Cannot connect to the IGT server!" << std::endl;
        std::cerr << "    <hostname> : IP or host name" << std::endl;
        std::cerr << "    <port>     : Port # (18944 in Slicer default)" << std::endl;
        exit(0);
    }

    // Allocate message class
    m_pointMsg_Tb1 = igtl::PointMessage::New();
    m_pointMsg_Tb1->SetDeviceName("IP0");
    m_pointMsg_Tb2 = igtl::PointMessage::New();
    m_pointMsg_Tb2->SetDeviceName("IP1");
    m_pointMsg_Tb3 = igtl::PointMessage::New();
    m_pointMsg_Tb3->SetDeviceName("IP2");

    // Allocate message classes for target and calyx points
    m_pointMsg_Target = igtl::PointMessage::New();
    m_pointMsg_Target->SetDeviceName("targetCoord");
    m_pointMsg_Calyx = igtl::PointMessage::New();
    m_pointMsg_Calyx->SetDeviceName("calyxCoord");

    // Allocate message classes for ellipsoid semi-axes points
    m_pointMsg_ellipsoidAxis = igtl::PointMessage::New();
    m_pointMsg_ellipsoidAxis->SetDeviceName("axis_Coord");
    

    // setting properties of target points
    m_pointerTarget = igtl::PointElement::New();
    m_pointerTarget->SetName("Target");
    m_pointerTarget->SetRGBA(0, 100, 0, 50); // green
    // setting properties of the renal calyx points
    m_pointerCalyx = igtl::PointElement::New();
    m_pointerCalyx->SetName("Renal_Calyx");
    m_pointerCalyx->SetRGBA(100, 0, 0, 50); // red
    // setting property of the ellipsoidal points
    for (size_t idx = 0UL; idx < 6UL; ++idx)
    {
        m_pointerEllipsoid[idx] = igtl::PointElement::New();
        m_pointerEllipsoid[idx]->SetRGBA(0, 0, 100, 50); // blue
    }
    // axes: 1-4, 2-5, 3-6
    m_pointerEllipsoid[0UL]->SetName("A1");
    m_pointerEllipsoid[1UL]->SetName("B1");
    m_pointerEllipsoid[2UL]->SetName("C1");
    m_pointerEllipsoid[3UL]->SetName("A2");
    m_pointerEllipsoid[4UL]->SetName("B2");
    m_pointerEllipsoid[5UL]->SetName("C2");

    // stringStream object for naming points in OpenIGTLink
    std::stringstream ss;

    // setting up the points
    for (size_t idx = 0UL; idx < 1000UL; ++idx)
    {
        m_pointersOfPoints_Tb1[idx] = igtl::PointElement::New();
        m_pointersOfPoints_Tb2[idx] = igtl::PointElement::New();
        m_pointersOfPoints_Tb3[idx] = igtl::PointElement::New();
        ss << "Point_" << std::to_string(idx);
        // clears the stringStream object
        ss.str(std::string());

        m_pointersOfPoints_Tb1[idx]->SetName(ss.str().c_str());
        m_pointersOfPoints_Tb1[idx]->SetGroupName("GROUP_0");
        m_pointersOfPoints_Tb1[idx]->SetRadius(0.50);
        m_pointersOfPoints_Tb1[idx]->SetOwner("IMAGE_0");
        m_pointersOfPoints_Tb1[idx]->SetRGBA(100, 0, 5, 15);

        m_pointersOfPoints_Tb2[idx]->SetName(ss.str().c_str());
        m_pointersOfPoints_Tb2[idx]->SetGroupName("GROUP_0");
        m_pointersOfPoints_Tb2[idx]->SetRadius(1.00);
        m_pointersOfPoints_Tb2[idx]->SetOwner("IMAGE_0");
        m_pointersOfPoints_Tb2[idx]->SetRGBA(0, 5, 100, 15);

        m_pointersOfPoints_Tb3[idx]->SetName(ss.str().c_str());
        m_pointersOfPoints_Tb3[idx]->SetGroupName("GROUP_0");
        m_pointersOfPoints_Tb3[idx]->SetRadius(1.50);
        m_pointersOfPoints_Tb3[idx]->SetOwner("IMAGE_0");
        m_pointersOfPoints_Tb3[idx]->SetRGBA(5, 100, 0, 15);
    }
}

// copy constructor
openIGTLink::openIGTLink(const openIGTLink &rhs) : m_hostname(rhs.m_hostname), m_port(rhs.m_port), m_socket(rhs.m_socket), m_pointMsg_Tb1(rhs.m_pointMsg_Tb1),
                                                   m_pointMsg_Tb2(rhs.m_pointMsg_Tb2), m_pointMsg_Tb3(rhs.m_pointMsg_Tb3), m_pointersOfPoints_Tb1(rhs.m_pointersOfPoints_Tb1),
                                                   m_pointersOfPoints_Tb2(rhs.m_pointersOfPoints_Tb2), m_pointersOfPoints_Tb3(rhs.m_pointersOfPoints_Tb3), m_pointMsg_Target(rhs.m_pointMsg_Target), m_pointMsg_Calyx(rhs.m_pointMsg_Calyx), m_pointerTarget(rhs.m_pointerTarget), m_pointerCalyx(rhs.m_pointerCalyx), m_pointMsg_ellipsoidAxis(rhs.m_pointMsg_ellipsoidAxis), m_pointerEllipsoid(rhs.m_pointerEllipsoid)
{
}

// move constructor
openIGTLink::openIGTLink(openIGTLink &&rhs) noexcept
{
    // handling self-assignment
    if (this != &rhs)
    {
        m_hostname = std::move(rhs.m_hostname);
        m_port = rhs.m_port;
        m_socket = std::move(rhs.m_socket);
        m_pointMsg_Tb1 = std::move(rhs.m_pointMsg_Tb1);
        m_pointMsg_Tb2 = std::move(rhs.m_pointMsg_Tb2);
        m_pointMsg_Tb3 = std::move(rhs.m_pointMsg_Tb3);
        m_pointersOfPoints_Tb1 = std::move(rhs.m_pointersOfPoints_Tb1);
        m_pointersOfPoints_Tb2 = std::move(rhs.m_pointersOfPoints_Tb2);
        m_pointersOfPoints_Tb3 = std::move(rhs.m_pointersOfPoints_Tb3);
        m_pointMsg_Target = std::move(rhs.m_pointMsg_Target);
        m_pointMsg_Calyx = std::move(rhs.m_pointMsg_Calyx);
        m_pointerTarget = std::move(rhs.m_pointerTarget);
        m_pointerCalyx = std::move(rhs.m_pointerCalyx);
        m_pointMsg_ellipsoidAxis = std::move(rhs.m_pointMsg_ellipsoidAxis);
        m_pointerEllipsoid = std::move(rhs.m_pointerEllipsoid);
    }
}

// Destructor
openIGTLink::~openIGTLink()
{
    this->m_socket->CloseSocket();
    std::cout << "Closing openIGTLink socket..." << std::endl;
}

// copy assignment operator
openIGTLink &openIGTLink::operator=(const openIGTLink &rhs)
{
    // handling self-assignment
    if (this != &rhs)
    {
        m_hostname = rhs.m_hostname;
        m_port = rhs.m_port;
        m_socket = rhs.m_socket;
        m_pointMsg_Tb1 = rhs.m_pointMsg_Tb1;
        m_pointMsg_Tb2 = rhs.m_pointMsg_Tb2;
        m_pointMsg_Tb3 = rhs.m_pointMsg_Tb3;
        m_pointersOfPoints_Tb1 = rhs.m_pointersOfPoints_Tb1;
        m_pointersOfPoints_Tb2 = rhs.m_pointersOfPoints_Tb2;
        m_pointersOfPoints_Tb3 = rhs.m_pointersOfPoints_Tb3;
        m_pointMsg_Target = rhs.m_pointMsg_Target;
        m_pointMsg_Calyx = rhs.m_pointMsg_Calyx;
        m_pointerTarget = rhs.m_pointerTarget;
        m_pointerCalyx = rhs.m_pointerCalyx;
        m_pointMsg_ellipsoidAxis = rhs.m_pointMsg_ellipsoidAxis;
        m_pointerEllipsoid = rhs.m_pointerEllipsoid;
    }

    return *this;
}

// move assignment operator
openIGTLink &openIGTLink::operator=(openIGTLink &&rhs) noexcept
{
    // handling self-assignment
    if (this != &rhs)
    {
        m_hostname = std::move(rhs.m_hostname);
        m_port = rhs.m_port;
        m_socket = std::move(rhs.m_socket);
        m_pointMsg_Tb1 = std::move(rhs.m_pointMsg_Tb1);
        m_pointMsg_Tb2 = std::move(rhs.m_pointMsg_Tb2);
        m_pointMsg_Tb3 = std::move(rhs.m_pointMsg_Tb3);
        m_pointersOfPoints_Tb1 = std::move(rhs.m_pointersOfPoints_Tb1);
        m_pointersOfPoints_Tb2 = std::move(rhs.m_pointersOfPoints_Tb2);
        m_pointersOfPoints_Tb3 = std::move(rhs.m_pointersOfPoints_Tb3);
        m_pointMsg_Target = std::move(rhs.m_pointMsg_Target);
        m_pointMsg_Calyx = std::move(rhs.m_pointMsg_Calyx);
        m_pointerTarget = std::move(rhs.m_pointerTarget);
        m_pointerCalyx = std::move(rhs.m_pointerCalyx);
        m_pointMsg_ellipsoidAxis = std::move(rhs.m_pointMsg_ellipsoidAxis);
        m_pointerEllipsoid = std::move(rhs.m_pointerEllipsoid);
    }

    return *this;
}

// Broadcasts points to 3D-Slicer
void openIGTLink::broadcastShapes(const blaze::HybridMatrix<double, 3UL, 1000UL, blaze::columnMajor> &Tb1, const blaze::HybridMatrix<double, 3UL, 1000UL, blaze::columnMajor> &Tb2, const blaze::HybridMatrix<double, 3UL, 1000UL, blaze::columnMajor> &Tb3)
{
    auto addPointElement = [&](const blaze::HybridMatrix<double, 3UL, 1000UL, blaze::columnMajor> &Tb, igtl::PointMessage::Pointer pointMsg, std::array<igtl::PointElement::Pointer, 1000UL> pointers, igtl::ClientSocket::Pointer socket)
    {
        pointMsg->ClearPointElement();
        for (size_t idx = 0UL; idx < Tb.columns(); ++idx)
        {
            pointers[idx]->SetPosition(Tb(0UL, idx), Tb(1UL, idx), Tb(2UL, idx));
            pointMsg->AddPointElement(pointers[idx]);
        }
        // pack points
        pointMsg->Pack();
        // Send points throgh the socket
        socket->Send(pointMsg->GetPackPointer(), pointMsg->GetPackSize());
    };

    addPointElement(Tb1, m_pointMsg_Tb1, m_pointersOfPoints_Tb1, m_socket);
    addPointElement(Tb2, m_pointMsg_Tb2, m_pointersOfPoints_Tb2, m_socket);
    addPointElement(Tb3, m_pointMsg_Tb3, m_pointersOfPoints_Tb3, m_socket);
}

// Broadcasts target point
void openIGTLink::broadcastTarget(const blaze::StaticVector<double, 3UL> &target)
{
    const blaze::StaticVector<double, 4UL> targetPatient = {target[0UL], target[1UL], target[2UL], 1.00};
    const blaze::StaticVector<double, 4UL> v = this->m_H_inv * targetPatient * 1.00E3;

    std::cout << "targetPatient = " << blaze::trans(targetPatient);
    std::cout << "v = " << blaze::trans(v);

    m_pointerTarget->SetPosition(v[0UL], v[1UL], v[2UL]);
    m_pointMsg_Target->ClearPointElement();
    m_pointMsg_Target->AddPointElement(m_pointerTarget);

    // pack points
    m_pointMsg_Target->Pack();
    // sends target point through socket
    m_socket->Send(m_pointMsg_Target->GetPackPointer(), m_pointMsg_Target->GetPackSize());
}

// Broadcasts calyx point
void openIGTLink::broadcastCalyx(const blaze::StaticVector<double, 3UL> &calyx)
{
    blaze::StaticVector<double, 4UL> v, calyxPatient = {calyx[0UL], calyx[1UL], calyx[2UL], 1.00};

    v = this->m_H_inv * calyxPatient * 1.00E3;
    m_pointerCalyx->SetPosition(v[0UL], v[1UL], v[2UL]);
    m_pointMsg_Calyx->ClearPointElement();
    m_pointMsg_Calyx->AddPointElement(m_pointerCalyx);

    // pack points
    m_pointMsg_Calyx->Pack();
    // sends calyx point through socket
    m_socket->Send(m_pointMsg_Calyx->GetPackPointer(), m_pointMsg_Calyx->GetPackSize());
}

// Broadcasts ellipsoidal target points
void openIGTLink::broadcastEllipsoidalTargets(const blaze::StaticMatrix<double, 3UL, 6UL> &target)
{
    blaze::StaticVector<double, 4UL> vec, tempTarget = {0.00, 0.00, 0.00, 1.00};

    m_pointMsg_ellipsoidAxis->ClearPointElement();
    for (size_t idx = 0UL; idx < 6UL; ++idx)
    {
        blaze::subvector<0UL, 3UL>(tempTarget) = blaze::column(target, idx);
        vec = this->m_H_inv * tempTarget;
        m_pointerEllipsoid[idx]->SetPosition(vec[0UL] * 1.00E3, vec[1UL] * 1.00e3, vec[2UL] * 1.00E3);
        m_pointMsg_ellipsoidAxis->AddPointElement(m_pointerEllipsoid[idx]);
    }
    // pack points
    m_pointMsg_ellipsoidAxis->Pack();
    // Send points throgh the socket
    m_socket->Send(m_pointMsg_ellipsoidAxis->GetPackPointer(), m_pointMsg_ellipsoidAxis->GetPackSize());
}

// sets the inverse homogeneous transformation (from CTR to Patient coordinate space)
void openIGTLink::setInverseTransformation(const blaze::StaticMatrix<double, 4UL, 4UL> &H_inv)
{
    this->m_H_inv = H_inv;
}