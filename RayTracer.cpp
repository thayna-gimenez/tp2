#include "graphics/Camera.h"
#include "utils/Stopwatch.h"
#include "RayTracer.h"
#include <iostream>

using namespace std;

namespace cg
{ // begin namespace cg

    namespace
    { // begin namespace

        inline void
            printElapsedTime(const char* s, Stopwatch::ms_time time)
        {
            printf("%sElapsed time: %g ms\n", s, time);
        }

        constexpr int MAX_STEPS = 4;
        constexpr int GRID_SIZE = MAX_STEPS + 1;
        constexpr float ADAPTIVE_DISTANCE = 0.0625f;

        struct B
        {
            Pixel p;
            uint8_t flag; // 0 = cru, 1 = cozido
        };

        B(*g_window)[GRID_SIZE] {};

        inline float
            arand()
        {
            return (float(std::rand()) / float(RAND_MAX) / 4.0f) - 0.125f;
        }

        inline Color
            pixelToColor(const Pixel& px)
        {
            Color c;
            c.r = float(px.r) / 255.0f;
            c.g = float(px.g) / 255.0f;
            c.b = float(px.b) / 255.0f;
            return c;
        }

    } // end namespace

    /////////////////////////////////////////////////////////////////////
    //
    // RayTracer implementation
    // =========
    RayTracer::RayTracer(SceneBase& scene, Camera& camera) :
        Renderer{ scene, camera },
        _maxRecursionLevel{ 6 },
        _minWeight{ minMinWeight },
        _useAdaptiveScan{ false }
    {
        // do nothing
    }

    void
        RayTracer::update()
    {
        // Delete current BVH before creating a new one
        _bvh = nullptr;

        PrimitiveBVH::PrimitiveArray primitives;
        auto np = uint32_t(0);

        primitives.reserve(_scene->actorCount());
        for (auto actor : _scene->actors())
            if (actor->visible)
            {
                auto p = actor->mapper()->primitive();

                assert(p != nullptr);
                if (p->canIntersect())
                {
                    primitives.push_back(p);
                    np++;
                }
            }
        _bvh = new PrimitiveBVH{ std::move(primitives) };
    }

    void
        RayTracer::render()
    {
        throw std::runtime_error("RayTracer::render() invoked");
    }

    void
        RayTracer::renderImage(Image& image)
    {
        Stopwatch timer;

        update();
        timer.start();
        {
            const auto& m = _camera->cameraToWorldMatrix();

            // VRC axes
            _vrc.u = m[0];
            _vrc.v = m[1];
            _vrc.n = m[2];
        }

        // init auxiliary mapping variables
        auto w = image.width(), h = image.height();

        setImageSize(w, h);
        _Iw = math::inverse(float(w));
        _Ih = math::inverse(float(h));
        {
            auto wh = _camera->windowHeight();

            if (w >= h)
                _Vw = (_Vh = wh) * w * _Ih;
            else
                _Vh = (_Vw = wh) * h * _Iw;
        }

        // init pixel ray
        float F, B;

        _camera->clippingPlanes(F, B);
        if (_camera->projectionType() == Camera::Perspective)
        {
            // distance from the camera position to a frustum back corner
            auto z = B / F * 0.5f;
            B = vec3f{ _Vw * z, _Vh * z, B }.length();
        }
        _pixelRay.tMin = F;
        _pixelRay.tMax = B;
        _pixelRay.set(_camera->position(), -_vrc.n);
        _numberOfRays = _numberOfHits = 0;
        if (_useAdaptiveScan)
            adaptiveScan(image);
        else
            scan(image);

        auto et = timer.time();

        std::cout << "\nNumber of rays: " << _numberOfRays;
        std::cout << "\nNumber of hits: " << _numberOfHits;
        printElapsedTime("\nDONE! ", et);
    }

    void
        RayTracer::setPixelRay(float x, float y)
        //[]---------------------------------------------------[]
        //|  Set pixel ray                                      |
        //|  @param x coordinate of the pixel                   |
        //|  @param y cordinates of the pixel                   |
        //[]---------------------------------------------------[]
    {
        auto p = imageToWindow(x, y);

        switch (_camera->projectionType())
        {
        case Camera::Perspective:
            _pixelRay.direction = (p - _camera->nearPlane() * _vrc.n).versor();
            break;

        case Camera::Parallel:
            _pixelRay.origin = _camera->position() + p;
            break;
        }
    }

    // Oversampler scan
    void
        RayTracer::adaptiveScan(Image& image)
    {
        const int w = _viewport.w;
        const int h = _viewport.h;
        ImageBuffer scanLine{ w, 1 };

        // buffer vertical: guarda a borda de baixo da linha anterior
        auto buffer = new B[w * GRID_SIZE];

        // inicializa buffer como "cru"
        for (int i = 0; i < w * GRID_SIZE; ++i)
            buffer[i].flag = 0;

        // janela deslizante de um pixel
        B window[GRID_SIZE][GRID_SIZE];

        for (int v = 0; v < GRID_SIZE; ++v)
            for (int u = 0; u < GRID_SIZE; ++u)
                window[v][u].flag = 0;

        // deixa a janela acess?vel para adaptiveShoot
        g_window = window;

        for (int j = 0; j < h; ++j) // percorre as linhas
        {
            // primeiro pixel da linha n?o tem vizinho ? esquerda
            for (int v = 0; v < GRID_SIZE; ++v)
                window[v][0].flag = 0;

            printf("Scanning line %d of %d\r", j + 1, h);

            for (int i = 0; i < w; ++i) // pixel por pixel na linha
            {
                const int base = i * GRID_SIZE;

                // 1) borda de cima vem do buffer (linha anterior)
                for (int u = 0; u < GRID_SIZE; ++u)
                    window[0][u] = buffer[base + u];

                // 2) interior da janela zera (reaproveitamos s? linha 0 e coluna 0)
                for (int v = 1; v < GRID_SIZE; ++v)
                    for (int u = 1; u < GRID_SIZE; ++u)
                        window[v][u].flag = 0;

                // 3) cor do pixel (i, j) via superamostragem adaptativa
                Color pixelColor = adaptiveShoot(0, 0, float(i), float(j), MAX_STEPS);
                scanLine[i] = pixelColor; // Pixel recebe Color pelo construtor

                // 4) borda de baixo vai para o buffer (para a pr?xima linha)
                for (int u = 0; u < GRID_SIZE; ++u)
                    buffer[base + u] = window[MAX_STEPS][u];

                // 5) borda da direita vira borda da esquerda do pr?ximo pixel da linha
                for (int v = 0; v < GRID_SIZE; ++v)
                    window[v][0] = window[v][MAX_STEPS];
            }
            image.setData(0, j, scanLine);
        }
        g_window = nullptr;
        delete[] buffer;
    }

    inline bool
        test(float a, float b, float t)
    {
        return math::abs(a - b) >= t;
    }

    inline bool
        test(const Color& c1, const Color& c2, float t)
    {
        return test(c1.r, c2.r, t) || test(c1.g, c2.g, t) || test(c1.b, c2.b, t);
    }

    Color
        RayTracer::adaptiveShoot(int i0, int j0, float x, float y, int step)
    {
        int i1 = i0 + step;
        int j1 = j0 + step;

        // helper: garante que (u, v) est? cozido e devolve a cor
        auto sampleAt = [&](int i, int j) -> Color
            {
                B& s = g_window[i][j]; // bolinha na posi??o (i,j)

                if (!s.flag) // se ? cru tra?amos novo raio
                {
                    // coordenadas normalizadas dentro do pixel [0,1] x [0,1]
                    const float dx = i / float(MAX_STEPS);
                    const float dy = j / float(MAX_STEPS);
                    // jitter aleat?rio dentro de ?1/8 do subpixel
                    const float px = x + dx + arand();
                    const float py = y + dy + arand();
                    Color c = shoot(px, py);

                    s.p = Pixel{ c };
                    s.flag = 1;
                    return c;
                }

                // j? cozido: converte Pixel de volta pra Color
                return pixelToColor(s.p);
            };

        // 4 cantos do bloco
        Color c0 = sampleAt(i0, j0); // sup. esq
        Color c1 = sampleAt(i1, j0); // sup. dir
        Color c2 = sampleAt(i0, j1); // inf. esq
        Color c3 = sampleAt(i1, j1); // inf. dir

        // m?dia Mp
        Color cm = (c0 + c1 + c2 + c3) * 0.25f;

        if (step == 1
            || !test(c0, cm, ADAPTIVE_DISTANCE)
            || !test(c1, cm, ADAPTIVE_DISTANCE)
            || !test(c2, cm, ADAPTIVE_DISTANCE)
            || !test(c3, cm, ADAPTIVE_DISTANCE)
            )
            return cm;

        // caso contr?rio, subdivide em 4 sub-blocos
        step >>= 1;
        i1 = i0 + step;
        j1 = j0 + step;
        // chama a fun??o pra cada sub-bloco:
        c0 = adaptiveShoot(i0, j0, x, y, step);
        c1 = adaptiveShoot(i1, j0, x, y, step);
        c2 = adaptiveShoot(i0, j1, x, y, step);
        c3 = adaptiveShoot(i1, j1, x, y, step);
        return (c0 + c1 + c2 + c3) * 0.25f;
    }
    
    void
        RayTracer::scan(Image& image)
    {
        ImageBuffer scanLine{ _viewport.w, 1 };

        for (auto j = 0; j < _viewport.h; j++)
        {
            auto y = (float)j + 0.5f;

            printf("Scanning line %d of %d\r", j + 1, _viewport.h);
            for (auto i = 0; i < _viewport.w; i++)
                scanLine[i] = shoot((float)i + 0.5f, y);
            image.setData(0, j, scanLine);
        }
    }

    Color
        RayTracer::shoot(float x, float y)
        //[]---------------------------------------------------[]
        //|  Shoot a pixel ray                                  |
        //|  @param x coordinate of the pixel                   |
        //|  @param y cordinates of the pixel                   |
        //|  @return RGB color of the pixel                     |
        //[]---------------------------------------------------[]
    {
        // set pixel ray
        setPixelRay(x, y);

        // trace pixel ray
        Color color = trace(_pixelRay, 0, 1, 1);

        // adjust RGB color
        if (color.r > 1.0f)
            color.r = 1.0f;
        if (color.g > 1.0f)
            color.g = 1.0f;
        if (color.b > 1.0f)
            color.b = 1.0f;
        // return pixel color
        return color;
    }

    Color
        RayTracer::trace(const Ray3f& ray, uint32_t level, float weight, float ior)
        //[]---------------------------------------------------[]
        //|  Trace a ray                                        |
        //|  @param the ray                                     |
        //|  @param recursion level                             |
        //|  @param ray weight                                  |
        //|  @return color of the ray                           |
        //[]---------------------------------------------------[]
    {
        if (level > _maxRecursionLevel)
            return Color::black;
        ++_numberOfRays;

        Intersection hit;

        return intersect(ray, hit) ? shade(ray, hit, level, weight, ior) : background();
    }

    inline constexpr auto
        rt_eps()
    {
        return 1e-4f;
    }

    bool
        RayTracer::intersect(const Ray3f& ray, Intersection& hit)
        //[]---------------------------------------------------[]
        //|  Ray/object intersection                            |
        //|  @param the ray (input)                             |
        //|  @param information on intersection (output)        |
        //|  @return true if the ray intersects an object       |
        //[]---------------------------------------------------[]
    {
        hit.object = nullptr;
        hit.distance = ray.tMax;
        return _bvh->intersect(ray, hit) ? ++_numberOfHits : false;
    }

    inline auto
        maxRGB(const Color& c)
    {
        return math::max(math::max(c.r, c.g), c.b);
    }

    Color
        RayTracer::shade(const Ray3f& ray,
            Intersection& hit,
            uint32_t level,
            float weight,
            float n1)
        //[]---------------------------------------------------[]
        //|  Shade a point P                                    |
        //|  @param the ray (input)                             |
        //|  @param information on intersection (input)         |
        //|  @param recursion level                             |
        //|  @param ray weight                                  |
        //|  @return color at point P                           |
        //[]---------------------------------------------------[]
    {
        auto primitive = (Primitive*)hit.object;

        assert(nullptr != primitive);

        auto N = primitive->normal(hit);
        const auto& V = ray.direction;
        auto NV = N.dot(V);

        // Make sure "real" normal is on right side
        if (NV > 0)
            N.negate(), NV = -NV;

        auto R = V - (2 * NV) * N; // reflection vector
        // Start with ambient lighting
        auto m = primitive->material();
        auto color = _scene->ambientLight * m->ambient;
        auto P = ray(hit.distance);

        // Compute direct lighting
        for (auto light : _scene->lights())
        {
            // If the light is turned off, then continue
            if (!light->isTurnedOn())
                continue;

            vec3f L;
            float d;

            // If the point P is out of the light range (for finite
            // point light or spotlight), then continue
            if (!light->lightVector(P, L, d))
                continue;

            auto NL = N.dot(L);

            // If light vector is backfaced, then continue
            if (NL <= 0)
                continue;

            auto lightRay = Ray3f{ P + L * rt_eps(), L };

            lightRay.tMax = d;
            ++_numberOfRays;
            // If the point P is shadowed, then continue
            if (shadow(lightRay))
                continue;

            auto lc = light->lightColor(d);

            color += lc * m->diffuse * NL;
            if (m->shine <= 0 || (d = R.dot(L)) <= 0)
                continue;
            color += lc * m->spot * pow(d, m->shine);
        }
        // Compute specular reflection
        if (m->specular != Color::black)
        {
            weight *= maxRGB(m->specular);
            if (weight > _minWeight && level < _maxRecursionLevel)
            {
                auto reflectionRay = Ray3f{ P + R * rt_eps(), R };
                color += m->specular * trace(reflectionRay, level + 1, weight, n1);
            }
        }

        // Compute specular refraction
        if (m->transparency != Color::black)
        {
            weight *= maxRGB(m->transparency);
            if (weight > _minWeight && level < _maxRecursionLevel)
            {
                auto n2 = m->ior;
                auto n12 = n1 / n2;
                auto c1 = -N.dot(V);
                auto c2 = 1 - (n12 * n12) * (1 - c1 * c1);

                if (c2 >= 0)
                {
                    auto T = n12 * V + (n12 * c1 - std::sqrt(c2)) * N;
                    auto refractionRay = Ray3f{ P + T * rt_eps(), T };
                    color += m->transparency * trace(refractionRay, level + 1, weight, n2);
                }
            }
        }
        return color;
    }

    Color
        RayTracer::background() const
        //[]---------------------------------------------------[]
        //|  Background                                         |
        //|  @return background color                           |
        //[]---------------------------------------------------[]
    {
        return _scene->backgroundColor;
    }

    bool
        RayTracer::shadow(Ray3f& ray)
        //[]---------------------------------------------------[]
        //|  Verifiy if ray is a shadow ray                     |
        //|  @param the ray (input)                             |
        //|  @return true if the ray intersects an object       |
        //[]---------------------------------------------------[]
    {
        for (Intersection hit; intersect(ray, hit);)
        {
            auto primitive = (Primitive*)hit.object;

            assert(nullptr != primitive);

            auto m = primitive->material();

            if (m->transparency == Color::black)
                return true;

            ray.origin = ray(hit.distance + rt_eps());
            ray.tMax -= hit.distance;
        }

        return false;
    }

} // end namespace cg
