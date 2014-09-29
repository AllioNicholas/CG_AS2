#pragma once

#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <iostream>

#ifndef M_PI
#define M_PI  3.14159265358979
#endif

// Inline functions to help with drawing
inline void glVertex( const FW::Vec3f& a )
{
    glVertex3fv(a.getPtr());
}

inline void glNormal( const FW::Vec3f& a ) 
{
    glNormal3fv(a.getPtr());
}

inline void glLoadMatrix( const FW::Mat4f& m )
{
    glLoadMatrixf( m.getPtr() );
}

inline void glMultMatrix( const FW::Mat4f& m )
{
    glMultMatrixf( m.getPtr() );
}

// Axis must be unit length. Angle in radians.
inline FW::Mat4f rotation4f( const FW::Vec3f& axis, float angle )
{
    FW::Mat3f R = FW::Mat3f::rotation(axis, angle);
    // Copy to homogeneous matrix
    FW::Mat4f H;
    H.setIdentity();
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            H(i, j) = R(i, j);
    return H;
}

inline void printTranspose(const FW::Vec3f& a)
{
    std::cout << a.x << " " << a.y << " " << a.z;
}

inline void printTranspose(const FW::Vec4f& a)
{
    std::cout << a.x << " " << a.y << " " << a.z << " " << a.w;
}

inline FW::Mat4f makeMat4f( float m00, float m01, float m02, float m03,
                            float m10, float m11, float m12, float m13,
                            float m20, float m21, float m22, float m23,
                            float m30, float m31, float m32, float m33)
{
    FW::Mat4f A;
    A.m00 = m00; A.m01 = m01; A.m02 = m02; A.m03 = m03;
    A.m10 = m10; A.m11 = m11; A.m12 = m12; A.m13 = m13;
    A.m20 = m20; A.m21 = m21; A.m22 = m22; A.m23 = m23;
    A.m30 = m30; A.m31 = m31; A.m32 = m32; A.m33 = m33;
    return A;
}

// Construct a matrix from the columns [a, b, c, d].
inline FW::Mat4f makeMat4f(const FW::Vec4f& a, const FW::Vec4f& b, const FW::Vec4f& c, const FW::Vec4f& d)
{
    FW::Mat4f A;
    A.col(0) = a;
    A.col(1) = b;
    A.col(2) = c;
    A.col(3) = d;
    return A;
}
