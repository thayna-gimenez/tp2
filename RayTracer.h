#ifndef __RayTracer_h
#define __RayTracer_h

#include "geometry/Intersection.h"
#include "graphics/Image.h"
#include "graphics/PrimitiveBVH.h"
#include "graphics/Renderer.h"

namespace cg
{ // begin namespace cg


/////////////////////////////////////////////////////////////////////
//
// RayTracer: simple ray tracer class
// =========
    class RayTracer : public Renderer
    {
    public:
        static constexpr auto minMinWeight = float(0.001);
        static constexpr auto maxMaxRecursionLevel = uint32_t(20);

        RayTracer(SceneBase&, Camera&);

        auto minWeight() const
        {
            return _minWeight;
        }

        void setMinWeight(float w)
        {
            _minWeight = math::max(w, minMinWeight);
        }

        auto maxRecursionLevel() const
        {
            return _maxRecursionLevel;
        }

        void setMaxRecursionLevel(uint32_t rl)
        {
            _maxRecursionLevel = math::min(rl, maxMaxRecursionLevel);
        }

        bool useAdaptiveScan() const
        {
            return _useAdaptiveScan;
        }

        void setUseAdaptiveScan(bool v)
        {
            _useAdaptiveScan = v;
        }

        void update() override;
        void render() override;
        virtual void renderImage(Image&);

    private:
        Reference<PrimitiveBVH> _bvh;
        struct VRC
        {
            vec3f u;
            vec3f v;
            vec3f n;

        } _vrc;
        float _minWeight;
        uint32_t _maxRecursionLevel;
        uint64_t _numberOfRays;
        uint64_t _numberOfHits;
        Ray3f _pixelRay;
        float _Vh;
        float _Vw;
        float _Ih;
        float _Iw;
        bool _useAdaptiveScan = false;

        void adaptiveScan(Image& image);
        Color adaptiveShoot(int i, int j, float x, float y, int step);
        void scan(Image& image);
        void setPixelRay(float x, float y);
        Color shoot(float x, float y);
        bool intersect(const Ray3f&, Intersection&);
        Color trace(const Ray3f& ray, uint32_t level, float weight, float ior);
        Color shade(const Ray3f&, Intersection&, uint32_t, float, float);
        bool shadow(Ray3f&);
        Color background() const;

        vec3f imageToWindow(float x, float y) const
        {
            return _Vw * (x * _Iw - 0.5f) * _vrc.u + _Vh * (y * _Ih - 0.5f) * _vrc.v;
        }

    }; // RayTracer

} // end namespace cg

#endif // __RayTracer_h