#pragma once

#include <blaze/Math.h>
#include <array>
#include <sstream>
#include "igtlOSUtil.h"
#include "igtlPointMessage.h"
#include "igtlClientSocket.h"


class openIGTLink
{
public:
    // default constructor
    openIGTLink() = delete;

    // overloaded constructor
    openIGTLink(char *hostname, int port);

    // copy constructor
    openIGTLink(const openIGTLink &rhs);

    // move constructor
    openIGTLink(openIGTLink &&rhs) noexcept;

    // CTR destructor
    ~openIGTLink();

    // copy assignment operator
    openIGTLink &operator=(const openIGTLink &rhs);

    // move assignment operator
    openIGTLink &operator=(openIGTLink &&rhs) noexcept;

    // Broadcasts points to 3D-Slicer
    void broadcastShapes(const blaze::HybridMatrix<double, 3UL, 1000UL, blaze::columnMajor> &Tb1, const blaze::HybridMatrix<double, 3UL, 1000UL, blaze::columnMajor> &Tb2, const blaze::HybridMatrix<double, 3UL, 1000UL, blaze::columnMajor> &Tb3);

    // Broadcasts target point
    void broadcastTarget(const blaze::StaticVector<double, 3UL>& target);

    // Broadcasts calyx point
    void broadcastCalyx(const blaze::StaticVector<double, 3UL>& calyx);

    // Broadcasts ellipsoidal target points
    void broadcastEllipsoidalTargets(const blaze::StaticMatrix<double, 3UL, 6UL> &target);

    // sets the inverse homogeneous transformation (from CTR to Patient coordinate space) 
    void setInverseTransformation(const blaze::StaticMatrix<double, 4UL, 4UL>& H_inv);

private:
    char *m_hostname;
    int m_port;
    // declaring an OpenIGTLink socket
    igtl::ClientSocket::Pointer m_socket;
    // declaring point messages for broadcasting the tube shapes individually
    igtl::PointMessage::Pointer m_pointMsg_Tb1, m_pointMsg_Tb2, m_pointMsg_Tb3;
    // array to store the point elements for the tubes' shapes
    std::array<igtl::PointElement::Pointer, 1000UL> m_pointersOfPoints_Tb1, m_pointersOfPoints_Tb2, m_pointersOfPoints_Tb3;

    // declaring point messages for broadcasting the targets (calyx + end-effector)
    igtl::PointMessage::Pointer m_pointMsg_Target, m_pointMsg_Calyx;
    igtl::PointElement::Pointer m_pointerTarget, m_pointerCalyx;
    
    // declaring point messages for broadcasting the ellipsoidal targets (extremeties of semi-axes)
    igtl::PointMessage::Pointer m_pointMsg_ellipsoidAxis;
    std::array<igtl::PointElement::Pointer, 6UL> m_pointerEllipsoid;
    
    blaze::StaticMatrix<double, 4UL, 4UL> m_H_inv;
};