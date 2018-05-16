/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#if !defined(__NORI_RAY_H)
#define __NORI_RAY_H

#include <nori/vector.h>
#include <memory>
#include <vector>

NORI_NAMESPACE_BEGIN

/**
 * \brief Simple n-dimensional ray segment data structure
 * 
 * Along with the ray origin and direction, this data structure additionally
 * stores a ray segment [mint, maxt] (whose entries may include positive/negative
 * infinity), as well as the componentwise reciprocals of the ray direction.
 * That is just done for convenience, as these values are frequently required.
 *
 * \remark Important: be careful when changing the ray direction. You must
 * call \ref update() to compute the componentwise reciprocals as well, or Nori's
 * ray-triangle intersection code will go haywire.
 */
template <typename _PointType, typename _VectorType> struct TRay {
    typedef _PointType                  PointType;
    typedef _VectorType                 VectorType;
    typedef typename PointType::Scalar  Scalar;

    PointType o;     ///< Ray origin
    VectorType d;    ///< Ray direction
    VectorType dRcp; ///< Componentwise reciprocals of the ray direction
    Scalar mint;     ///< Minimum position on the ray segment
    Scalar maxt;     ///< Maximum position on the ray segment

    /// Construct a new ray
    TRay() : mint(Epsilon), 
        maxt(std::numeric_limits<Scalar>::infinity()) { }
    
    /// Construct a new ray
    TRay(const PointType &o, const VectorType &d) : o(o), d(d), 
            mint(Epsilon), maxt(std::numeric_limits<Scalar>::infinity()) {
        update();
    }

    /// Construct a new ray
    TRay(const PointType &o, const VectorType &d, 
        Scalar mint, Scalar maxt) : o(o), d(d), mint(mint), maxt(maxt) {
        update();
    }

    /// Copy constructor
    TRay(const TRay &ray) 
     : o(ray.o), d(ray.d), dRcp(ray.dRcp),
       mint(ray.mint), maxt(ray.maxt) { }

    /// Copy a ray, but change the covered segment of the copy
    TRay(const TRay &ray, Scalar mint, Scalar maxt) 
     : o(ray.o), d(ray.d), dRcp(ray.dRcp), mint(mint), maxt(maxt) { }

    /// Update the reciprocal ray directions after changing 'd'
    void update() {
        dRcp = d.cwiseInverse();
    }

    /// Return the position of a point along the ray
    PointType operator() (Scalar t) const { return o + t * d; }

    /// Return a ray that points into the opposite direction
    TRay reverse() const {
        TRay result;
        result.o = o; result.d = -d; result.dRcp = -dRcp;
        result.mint = mint; result.maxt = maxt;
        return result;
    }

    /// Return a human-readable string summary of this ray
    std::string toString() const {
        return tfm::format(
                "Ray[\n"
                "  o = %s,\n"
                "  d = %s,\n"
                "  mint = %f,\n"
                "  maxt = %f\n"
                "]", o.toString(), d.toString(), mint, maxt);
    }
};


template<typename _PointType, typename _VectorType> struct TRayDifferential 
    : public TRay<_PointType, _VectorType>
{
    typedef _PointType                  PointType;
    typedef _VectorType                 VectorType;
    typedef typename PointType::Scalar  Scalar;

    TRayDifferential() : TRay() {
        m_quality = 0;
        m_hasRayDifferentials = false;
    }

    TRayDifferential(int quality) : TRay() {
        m_quality = quality;
    }

    TRayDifferential(const PointType &o, const VectorType &d) : TRay(o, d) {}
    TRayDifferential(const TRay &ray) : TRay(ray) {}
    TRayDifferential(const TRay &ray, Scalar mint, Scalar maxt) : TRay(ray, mint, maxt) {}
    TRay<_PointType, _VectorType> getRay() const { return TRay(o, d, mint, maxt); }
    
    void setStencilRay(const int index, const TRay<_PointType, _VectorType>& ray) {
        m_stencilRays.at(index) = ray;
    }
    
    const TRay<_PointType, _VectorType>& getStencilRay(const int index) const {
        return (m_stencilRays.at(index));
    }
    
    // We use these two methods to initialize the ray stencils
    void setQuality(int quality) {
        m_quality = quality;
        setupRayDifferential();
    }
    void setupRayDifferential() {
        m_totalStencilRays = 4 * m_quality * (m_quality + 1);
        m_stencilRays.resize(m_totalStencilRays);
        m_hasRayDifferentials = true;
    }

    // We have an additional circle stencil of rays
    // each circle has 8 rays
    // so total number of rays is 8 * n(n+1)/2
    // plus the additional central ray
    std::vector<TRay<_PointType, _VectorType>> m_stencilRays;
    int m_quality;
    int m_totalStencilRays;
    bool m_hasRayDifferentials;
};

NORI_NAMESPACE_END

#endif /* __NORI_RAY_H */

