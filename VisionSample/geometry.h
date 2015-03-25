
/*
    pbrt source code Copyright(c) 1998-2012 Matt Pharr and Greg Humphreys.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_CORE_GEOMETRY_H
#define PBRT_CORE_GEOMETRY_H

// core/geometry.h*
#include "pbrt.h"

// Geometry Declarations
class Vector {
public:
    // Vector Public Methods
    Vector() { x = y = z = 0.f; }
    Vector(double xx, double yy, double zz)
        : x(xx), y(yy), z(zz) {
        assert(!HasNaNs());
    }
    bool HasNaNs() const { return isnan(x) || isnan(y) || isnan(z); }
    explicit Vector(const Point &p);
#ifndef NDEBUG
    // The default versions of these are fine for release builds; for debug
    // we define them so that we can add the assert checks.
    Vector(const Vector &v) {
        assert(!v.HasNaNs());
        x = v.x; y = v.y; z = v.z;
    }
    
    Vector &operator=(const Vector &v) {
        assert(!v.HasNaNs());
        x = v.x; y = v.y; z = v.z;
        return *this;
    }
#endif // !NDEBUG
    Vector operator+(const Vector &v) const {
        assert(!v.HasNaNs());
        return Vector(x + v.x, y + v.y, z + v.z);
    }
    
    Vector& operator+=(const Vector &v) {
        assert(!v.HasNaNs());
        x += v.x; y += v.y; z += v.z;
        return *this;
    }
    Vector operator-(const Vector &v) const {
        assert(!v.HasNaNs());
        return Vector(x - v.x, y - v.y, z - v.z);
    }
    
    Vector& operator-=(const Vector &v) {
        assert(!v.HasNaNs());
        x -= v.x; y -= v.y; z -= v.z;
        return *this;
    }
    Vector operator*(double f) const { return Vector(f*x, f*y, f*z); }
    
    Vector &operator*=(double f) {
        assert(!isnan(f));
        x *= f; y *= f; z *= f;
        return *this;
    }
    Vector operator/(double f) const {
        assert(f != 0);
        double inv = 1.f / f;
        return Vector(x * inv, y * inv, z * inv);
    }
    
    Vector &operator/=(double f) {
        assert(f != 0);
        double inv = 1.f / f;
        x *= inv; y *= inv; z *= inv;
        return *this;
    }
    Vector operator-() const { return Vector(-x, -y, -z); }
    double operator[](int i) const {
        assert(i >= 0 && i <= 2);
        return (&x)[i];
    }
    
    double &operator[](int i) {
        assert(i >= 0 && i <= 2);
        return (&x)[i];
    }
    double LengthSquared() const { return x*x + y*y + z*z; }
    double Length() const { return sqrtf(LengthSquared()); }
    explicit Vector(const Normal &n);

    bool operator==(const Vector &v) const {
        return x == v.x && y == v.y && z == v.z;
    }
    bool operator!=(const Vector &v) const {
        return x != v.x || y != v.y || z != v.z;
    }

    // Vector Public Data
    double x, y, z;
};


class Point {
public:
    // Point Public Methods
    Point() { x = y = z = 0.f; }
    Point(double xx, double yy, double zz)
        : x(xx), y(yy), z(zz) {
        assert(!HasNaNs());
    }
#ifndef NDEBUG
    Point(const Point &p) {
        assert(!p.HasNaNs());
        x = p.x; y = p.y; z = p.z;
    }
    
    Point &operator=(const Point &p) {
        assert(!p.HasNaNs());
        x = p.x; y = p.y; z = p.z;
        return *this;
    }
#endif // !NDEBUG
    Point operator+(const Vector &v) const {
        assert(!v.HasNaNs());
        return Point(x + v.x, y + v.y, z + v.z);
    }
    
    Point &operator+=(const Vector &v) {
        assert(!v.HasNaNs());
        x += v.x; y += v.y; z += v.z;
        return *this;
    }
    Vector operator-(const Point &p) const {
        assert(!p.HasNaNs());
        return Vector(x - p.x, y - p.y, z - p.z);
    }
    
    Point operator-(const Vector &v) const {
        assert(!v.HasNaNs());
        return Point(x - v.x, y - v.y, z - v.z);
    }
    
    Point &operator-=(const Vector &v) {
        assert(!v.HasNaNs());
        x -= v.x; y -= v.y; z -= v.z;
        return *this;
    }
    Point &operator+=(const Point &p) {
        assert(!p.HasNaNs());
        x += p.x; y += p.y; z += p.z;
        return *this;
    }
    Point operator+(const Point &p) const {
        assert(!p.HasNaNs());
        return Point(x + p.x, y + p.y, z + p.z);
    }
    Point operator* (double f) const {
        return Point(f*x, f*y, f*z);
    }
    Point &operator*=(double f) {
        x *= f; y *= f; z *= f;
        return *this;
    }
    Point operator/ (double f) const {
        double inv = 1.f/f;
        return Point(inv*x, inv*y, inv*z);
    }
    Point &operator/=(double f) {
        double inv = 1.f/f;
        x *= inv; y *= inv; z *= inv;
        return *this;
    }
    double operator[](int i) const {
        assert(i >= 0 && i <= 2);
        return (&x)[i];
    }
    
    double &operator[](int i) {
        assert(i >= 0 && i <= 2);
        return (&x)[i];
    }
    bool HasNaNs() const {
        return isnan(x) || isnan(y) || isnan(z);
    }

    bool operator==(const Point &p) const {
        return x == p.x && y == p.y && z == p.z;
    }
    bool operator!=(const Point &p) const {
        return x != p.x || y != p.y || z != p.z;
    }

    // Point Public Data
    double x, y, z;
};


class Normal {
public:
    // Normal Public Methods
    Normal() { x = y = z = 0.f; }
    Normal(double xx, double yy, double zz)
        : x(xx), y(yy), z(zz) {
        assert(!HasNaNs());
    }
    Normal operator-() const {
        return Normal(-x, -y, -z);
    }
    Normal operator+ (const Normal &n) const {
        assert(!n.HasNaNs());
        return Normal(x + n.x, y + n.y, z + n.z);
    }
    
    Normal& operator+=(const Normal &n) {
        assert(!n.HasNaNs());
        x += n.x; y += n.y; z += n.z;
        return *this;
    }
    Normal operator- (const Normal &n) const {
        assert(!n.HasNaNs());
        return Normal(x - n.x, y - n.y, z - n.z);
    }
    
    Normal& operator-=(const Normal &n) {
        assert(!n.HasNaNs());
        x -= n.x; y -= n.y; z -= n.z;
        return *this;
    }
    bool HasNaNs() const {
        return isnan(x) || isnan(y) || isnan(z);
    }
    Normal operator*(double f) const {
        return Normal(f*x, f*y, f*z);
    }
    
    Normal &operator*=(double f) {
        x *= f; y *= f; z *= f;
        return *this;
    }
    Normal operator/(double f) const {
        assert(f != 0);
        double inv = 1.f/f;
        return Normal(x * inv, y * inv, z * inv);
    }
    
    Normal &operator/=(double f) {
        assert(f != 0);
        double inv = 1.f/f;
        x *= inv; y *= inv; z *= inv;
        return *this;
    }
    double LengthSquared() const { return x*x + y*y + z*z; }
    double Length() const        { return sqrtf(LengthSquared()); }
    
#ifndef NDEBUG
    Normal(const Normal &n) {
        assert(!n.HasNaNs());
        x = n.x; y = n.y; z = n.z;
    }
    
    Normal &operator=(const Normal &n) {
        assert(!n.HasNaNs());
        x = n.x; y = n.y; z = n.z;
        return *this;
    }
#endif // !NDEBUG
    explicit Normal(const Vector &v)
      : x(v.x), y(v.y), z(v.z) {
        assert(!v.HasNaNs());
    }
    double operator[](int i) const {
        assert(i >= 0 && i <= 2);
        return (&x)[i];
    }
    
    double &operator[](int i) {
        assert(i >= 0 && i <= 2);
        return (&x)[i];
    }

    bool operator==(const Normal &n) const {
        return x == n.x && y == n.y && z == n.z;
    }
    bool operator!=(const Normal &n) const {
        return x != n.x || y != n.y || z != n.z;
    }

    // Normal Public Data
    double x, y, z;
};


class Ray {
public:
    // Ray Public Methods
    Ray() : mint(0.f), maxt(INFINITY), time(0.f), depth(0) { }
	Ray(const Point &origin, const Vector &direction)
		: o(origin), d(direction), mint(0), maxt(INFINITY), time(0.f), depth(0) { }
    Ray(const Point &origin, const Vector &direction,
        double start, double end = INFINITY, double t = 0.f, int d = 0)
        : o(origin), d(direction), mint(start), maxt(end), time(t), depth(d) { }
    Ray(const Point &origin, const Vector &direction, const Ray &parent,
        double start, double end = INFINITY)
        : o(origin), d(direction), mint(start), maxt(end),
          time(parent.time), depth(parent.depth+1) { }
    Point operator()(double t) const { return o + d * t; }
    bool HasNaNs() const {
        return (o.HasNaNs() || d.HasNaNs() ||
                isnan(mint) || isnan(maxt));
    }

    // Ray Public Data
    Point o;
    Vector d;
    mutable double mint, maxt;
    double time;
    int depth;
};



// Geometry Inline Functions
inline Vector::Vector(const Point &p)
    : x(p.x), y(p.y), z(p.z) {
    assert(!HasNaNs());
}


inline Vector operator*(double f, const Vector &v) { return v*f; }
inline double Dot(const Vector &v1, const Vector &v2) {
    assert(!v1.HasNaNs() && !v2.HasNaNs());
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}


inline double AbsDot(const Vector &v1, const Vector &v2) {
    assert(!v1.HasNaNs() && !v2.HasNaNs());
    return fabsf(Dot(v1, v2));
}


inline Vector Cross(const Vector &v1, const Vector &v2) {
    assert(!v1.HasNaNs() && !v2.HasNaNs());
    double v1x = v1.x, v1y = v1.y, v1z = v1.z;
    double v2x = v2.x, v2y = v2.y, v2z = v2.z;
    return Vector((v1y * v2z) - (v1z * v2y),
                  (v1z * v2x) - (v1x * v2z),
                  (v1x * v2y) - (v1y * v2x));
}


inline Vector Cross(const Vector &v1, const Normal &v2) {
    assert(!v1.HasNaNs() && !v2.HasNaNs());
    double v1x = v1.x, v1y = v1.y, v1z = v1.z;
    double v2x = v2.x, v2y = v2.y, v2z = v2.z;
    return Vector((v1y * v2z) - (v1z * v2y),
                  (v1z * v2x) - (v1x * v2z),
                  (v1x * v2y) - (v1y * v2x));
}


inline Vector Cross(const Normal &v1, const Vector &v2) {
    assert(!v1.HasNaNs() && !v2.HasNaNs());
    double v1x = v1.x, v1y = v1.y, v1z = v1.z;
    double v2x = v2.x, v2y = v2.y, v2z = v2.z;
    return Vector((v1y * v2z) - (v1z * v2y),
                  (v1z * v2x) - (v1x * v2z),
                  (v1x * v2y) - (v1y * v2x));
}


inline Vector Normalize(const Vector &v) { return v / v.Length(); }
inline void CoordinateSystem(const Vector &v1, Vector *v2, Vector *v3) {
    if (fabsf(v1.x) > fabsf(v1.y)) {
        double invLen = 1.f / sqrtf(v1.x*v1.x + v1.z*v1.z);
        *v2 = Vector(-v1.z * invLen, 0.f, v1.x * invLen);
    }
    else {
        double invLen = 1.f / sqrtf(v1.y*v1.y + v1.z*v1.z);
        *v2 = Vector(0.f, v1.z * invLen, -v1.y * invLen);
    }
    *v3 = Cross(v1, *v2);
}


inline double Distance(const Point &p1, const Point &p2) {
    return (p1 - p2).Length();
}


inline double DistanceSquared(const Point &p1, const Point &p2) {
    return (p1 - p2).LengthSquared();
}


inline Point operator*(double f, const Point &p) {
    assert(!p.HasNaNs());
    return p*f;
}


inline Normal operator*(double f, const Normal &n) {
    return Normal(f*n.x, f*n.y, f*n.z);
}


inline Normal Normalize(const Normal &n) {
    return n / n.Length();
}


inline Vector::Vector(const Normal &n)
  : x(n.x), y(n.y), z(n.z) {
    assert(!n.HasNaNs());
}


inline double Dot(const Normal &n1, const Vector &v2) {
    assert(!n1.HasNaNs() && !v2.HasNaNs());
    return n1.x * v2.x + n1.y * v2.y + n1.z * v2.z;
}


inline double Dot(const Vector &v1, const Normal &n2) {
    assert(!v1.HasNaNs() && !n2.HasNaNs());
    return v1.x * n2.x + v1.y * n2.y + v1.z * n2.z;
}


inline double Dot(const Normal &n1, const Normal &n2) {
    assert(!n1.HasNaNs() && !n2.HasNaNs());
    return n1.x * n2.x + n1.y * n2.y + n1.z * n2.z;
}


inline double AbsDot(const Normal &n1, const Vector &v2) {
    assert(!n1.HasNaNs() && !v2.HasNaNs());
    return fabsf(n1.x * v2.x + n1.y * v2.y + n1.z * v2.z);
}


inline double AbsDot(const Vector &v1, const Normal &n2) {
    assert(!v1.HasNaNs() && !n2.HasNaNs());
    return fabsf(v1.x * n2.x + v1.y * n2.y + v1.z * n2.z);
}


inline double AbsDot(const Normal &n1, const Normal &n2) {
    assert(!n1.HasNaNs() && !n2.HasNaNs());
    return fabsf(n1.x * n2.x + n1.y * n2.y + n1.z * n2.z);
}


inline Normal Faceforward(const Normal &n, const Vector &v) {
    return (Dot(n, v) < 0.f) ? -n : n;
}


inline Normal Faceforward(const Normal &n, const Normal &n2) {
    return (Dot(n, n2) < 0.f) ? -n : n;
}



inline Vector Faceforward(const Vector &v, const Vector &v2) {
    return (Dot(v, v2) < 0.f) ? -v : v;
}



inline Vector Faceforward(const Vector &v, const Normal &n2) {
    return (Dot(v, n2) < 0.f) ? -v : v;
}


inline Vector SphericalDirection(double sintheta,
                                 double costheta, double phi) {
    return Vector(sintheta * cosf(phi),
                  sintheta * sinf(phi),
                  costheta);
}


inline Vector SphericalDirection(double sintheta, double costheta,
                                 double phi, const Vector &x,
                                 const Vector &y, const Vector &z) {
    return sintheta * cosf(phi) * x +
           sintheta * sinf(phi) * y + costheta * z;
}


inline double SphericalTheta(const Vector &v) {
    return acosf(Clamp(v.z, -1.f, 1.f));
}


inline double SphericalPhi(const Vector &v) {
    double p = atan2f(v.y, v.x);
    return (p < 0.f) ? p + 2.f*M_PI : p;
}



#endif // PBRT_CORE_GEOMETRY_H
