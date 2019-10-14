#include <cstdio> 
#include <cstdlib> 
#include <memory> 
#include <vector> 
#include <utility> 
#include <cstdint> 
#include <iostream> 
#include <fstream> 
#include <cmath> 

#ifndef UTILH_INCLUDE
#define UTILH_INCLUDE

const float kInfinity = std::numeric_limits<float>::max();
 
class Vec3f { 
public: 
    Vec3f() : x(0), y(0), z(0) {} 
    Vec3f(float xx) : x(xx), y(xx), z(xx) {} 
    Vec3f(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {} 
    Vec3f operator * (const float &r) const { return Vec3f(x * r, y * r, z * r); } 
    Vec3f operator * (const Vec3f &v) const { return Vec3f(x * v.x, y * v.y, z * v.z); } 
    Vec3f operator - (const Vec3f &v) const { return Vec3f(x - v.x, y - v.y, z - v.z); } 
    Vec3f operator + (const Vec3f &v) const { return Vec3f(x + v.x, y + v.y, z + v.z); } 
    Vec3f operator + (const float &r) const { return Vec3f(x + r, y + r, z + r); } 
    Vec3f operator - () const { return Vec3f(-x, -y, -z); } 
    Vec3f& operator += (const Vec3f &v) { x += v.x, y += v.y, z += v.z; return *this; }
    bool operator==(const Vec3f &v) const { return v.x == x && v.y == y && v.z == z; }
    bool operator!=(const Vec3f &v) const { return v.x != x || v.y != y || v.z != z; }
    inline float operator[](int i) const { 
        if (i == 0)
            return x;
        else if(i == 1)
            return y;
        else
            return z;
    }
    inline float &operator[](int i) { 
        if (i == 0)
            return x;
        else if(i == 1)
            return y;
        else
            return z;
    }
    friend Vec3f operator * (const float &r, const Vec3f &v) 
    { return Vec3f(v.x * r, v.y * r, v.z * r); } 
    friend std::ostream & operator << (std::ostream &os, const Vec3f &v) 
    { return os << v.x << ", " << v.y << ", " << v.z; } 
    float x, y, z; 
};

Vec3f Min(const Vec3f &p1, const Vec3f &p2) {
    return Vec3f(std::min(p1.x, p2.x), std::min(p1.y, p2.y),
                 std::min(p1.z, p2.z));
}

Vec3f Max(const Vec3f &p1, const Vec3f &p2) {
    return Vec3f(std::max(p1.x, p2.x), std::max(p1.y, p2.y),
                 std::max(p1.z, p2.z));
}
 
class Vec2f 
{ 
public: 
    Vec2f() : x(0), y(0) {} 
    Vec2f(float xx) : x(xx), y(xx) {} 
    Vec2f(float xx, float yy) : x(xx), y(yy) {} 
    Vec2f operator * (const float &r) const { return Vec2f(x * r, y * r); } 
    Vec2f operator + (const Vec2f &v) const { return Vec2f(x + v.x, y + v.y); } 
    float x, y; 
}; 
 
Vec3f normalize(const Vec3f &v) 
{ 
    float mag2 = v.x * v.x + v.y * v.y + v.z * v.z; 
    if (mag2 > 0) { 
        float invMag = 1 / sqrtf(mag2); 
        return Vec3f(v.x * invMag, v.y * invMag, v.z * invMag); 
    } 
 
    return v; 
} 
 
inline 
float dotProduct(const Vec3f &a, const Vec3f &b) 
{ return a.x * b.x + a.y * b.y + a.z * b.z; } 
 
Vec3f crossProduct(const Vec3f &a, const Vec3f &b) 
{ 
    return Vec3f( 
        a.y * b.z - a.z * b.y, 
        a.z * b.x - a.x * b.z, 
        a.x * b.y - a.y * b.x 
    ); 
} 
 
inline 
float clamp(const float &lo, const float &hi, const float &v) 
{ return std::max(lo, std::min(hi, v)); } 
 
inline 
float deg2rad(const float &deg) 
{ return deg * M_PI / 180; } 
 
inline 
Vec3f mix(const Vec3f &a, const Vec3f& b, const float &mixValue) 
{ return a * (1 - mixValue) + b * mixValue; } 

class Bounds3f{
public:
    Bounds3f() {
        float minNum = std::numeric_limits<float>::lowest();
        float maxNum = std::numeric_limits<float>::max();
        pMin = Vec3f(maxNum, maxNum, maxNum);
        pMax = Vec3f(minNum, minNum, minNum);
    }
    explicit Bounds3f(const Vec3f &p) : pMin(p), pMax(p) {}
    Bounds3f(const Vec3f &p1, const Vec3f &p2) 
        : pMin(std::min(p1.x, p2.x), std::min(p1.y,p2.y),
               std::min(p1.z, p2.z)),
          pMax(std::max(p1.x, p2.x), std::max(p1.y,p2.y),
               std::max(p1.z, p2.z)) {}
    bool operator==(const Bounds3f &b) const {
        return b.pMin == pMin && b.pMax == pMax;
    }
    bool operator!=(const Bounds3f &b) const {
        return b.pMin != pMin || b.pMax != pMax;
    }
    inline const Vec3f operator[](int i) const;

    Vec3f Diagonal() const { return pMax - pMin; }
    int MaximumExtent() const {
        Vec3f d = Diagonal();
        if (d.x > d.y && d.x > d.z) 
            return 0;
        else if (d.y > d.z)
            return 1;
        else 
            return 2;
    }

    // Only return whether intersects or not
    // Could add t_position of intersections
    inline bool intersect(const Vec3f &orig, const Vec3f &dir) const { 
        float t0 = 0, t1 = kInfinity;
        for (int i = 0; i < 3; ++i) {
            // Find interval of planes
            float invRayDir = 1 / dir[i];
            float tNear = (pMin[i] - orig[i]) * invRayDir;
            float tFar = (pMax[i] - orig[i]) * invRayDir;

            if (tNear > tFar) std::swap(tNear, tFar);

            // Do something to ensure the accuracy of tFar
            tFar *= 1 + 2 * tgamma(3);
            t0 = tNear > t0 ? tNear : t0;
            t1 = tFar < t1 ? tFar : t1;
            if (t0 > t1) return false;
        }
        return true;
    }

    friend std::ostream &operator<<(std::ostream &os, const Bounds3f &b) {
         os << "[ " << b.pMin << " - " << b.pMax << " ]";
         return os;
    }

    Vec3f pMin, pMax;
};

Bounds3f Union(const Bounds3f &b1, const Bounds3f &b2) {
    Bounds3f ret;
    ret.pMin = Min(b1.pMin, b2.pMin);
    ret.pMax = Max(b1.pMax, b2.pMax);
    /*std::cout << "Union()" << std::endl;
    std::cout << b1 << std::endl;
    std::cout << b2 << std::endl;
    std::cout << ret << std::endl;*/
    return ret;
}

Bounds3f Union(const Bounds3f &b, const Vec3f &v) {
    Bounds3f ret;
    ret.pMin = Min(b.pMin, v);
    ret.pMax = Max(b.pMax, v);
    return ret;
}
 
struct Options 
{ 
    uint32_t width; 
    uint32_t height; 
    float fov; 
    float imageAspectRatio; 
    uint8_t maxDepth; 
    Vec3f backgroundColor; 
    float bias; 
}; 
 
class Light 
{ 
public: 
    Light(const Vec3f &p, const Vec3f &i) : position(p), intensity(i) {} 
    Vec3f position; 
    Vec3f intensity; 
}; 
 
enum MaterialType { DIFFUSE_AND_GLOSSY, REFLECTION_AND_REFRACTION, REFLECTION }; 
 
class Object 
{ 
 public: 
    Object() : 
        materialType(DIFFUSE_AND_GLOSSY), 
        ior(1.3), Kd(0.8), Ks(0.2), diffuseColor(0.2), specularExponent(25), isMesh(false) {} 
    virtual ~Object() {} 
    virtual bool intersect(const Vec3f &, const Vec3f &, float &, uint32_t &, Vec2f &) const = 0; 
    virtual void getSurfaceProperties(const Vec3f &, const Vec3f &, const uint32_t &, const Vec2f &, Vec3f &, Vec2f &) const = 0; 
    virtual Vec3f evalDiffuseColor(const Vec2f &) const { return diffuseColor; } 
    virtual Bounds3f objectBounds() const = 0;
    // material properties
    MaterialType materialType; 
    float ior; 
    float Kd, Ks; 
    Vec3f diffuseColor; 
    float specularExponent;
    bool isMesh;
}; 
 
bool solveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1) 
{ 
    float discr = b * b - 4 * a * c; 
    if (discr < 0) return false; 
    else if (discr == 0) x0 = x1 = - 0.5 * b / a; 
    else { 
        float q = (b > 0) ? 
            -0.5 * (b + sqrt(discr)) : 
            -0.5 * (b - sqrt(discr)); 
        x0 = q / a; 
        x1 = c / q; 
    } 
    if (x0 > x1) std::swap(x0, x1); 
    return true; 
} 
 
class Sphere : public Object{ 
public: 
    Sphere(const Vec3f &c, const float &r) : center(c), radius(r), radius2(r * r) {} 
    bool intersect(const Vec3f &orig, const Vec3f &dir, float &tnear, uint32_t &index, Vec2f &uv) const 
    { 
        // analytic solution
        Vec3f L = orig - center; 
        float a = dotProduct(dir, dir); 
        float b = 2 * dotProduct(dir, L); 
        float c = dotProduct(L, L) - radius2; 
        float t0, t1; 
        if (!solveQuadratic(a, b, c, t0, t1)) return false; 
        if (t0 < 0) t0 = t1; 
        if (t0 < 0) return false; 
        tnear = t0; 
 
        return true; 
    } 
 
    void getSurfaceProperties(const Vec3f &P, const Vec3f &I, const uint32_t &index, const Vec2f &uv, Vec3f &N, Vec2f &st) const 
    { N = normalize(P - center); } 

    Bounds3f objectBounds() const {
        /*
        std::cout << "objectBounds()" << std::endl;
        std::cout << center << std::endl;
        std::cout << radius << std::endl;
        std::cout << Vec3f(center.x - radius, center.y - radius, center.z - radius) << std::endl;
        std::cout << Vec3f(center.x + radius, center.y + radius, center.z + radius) << std::endl;
        */
        return Bounds3f(Vec3f(center.x - radius, center.y - radius, center.z - radius), 
                        Vec3f(center.x + radius, center.y + radius, center.z + radius));
    }
 
    Vec3f center; 
    float radius, radius2; 
};

 
bool rayTriangleIntersect( 
    const Vec3f &v0, const Vec3f &v1, const Vec3f &v2, 
    const Vec3f &orig, const Vec3f &dir, 
    float &tnear, float &u, float &v) 
{ 
    Vec3f edge1 = v1 - v0; 
    Vec3f edge2 = v2 - v0; 
    Vec3f pvec = crossProduct(dir, edge2); 
    float det = dotProduct(edge1, pvec); 
    if (det == 0 || det < 0) return false; 
 
    Vec3f tvec = orig - v0; 
    u = dotProduct(tvec, pvec); 
    if (u < 0 || u > det) return false; 
 
    Vec3f qvec = crossProduct(tvec, edge1); 
    v = dotProduct(dir, qvec); 
    if (v < 0 || u + v > det) return false; 
 
    float invDet = 1 / det; 
 
    tnear = dotProduct(edge2, qvec) * invDet; 
    u *= invDet; 
    v *= invDet; 
 
    return true; 
} 

class Triangle : public Object{
public:
    Triangle(const Vec3f &_v0, const Vec3f &_v1, const Vec3f &_v2) : v0(_v0), v1(_v1), v2(_v2) {}

    bool intersect(const Vec3f &orig, const Vec3f &dir, float &tnear, uint32_t &index, Vec2f &uv) const
    {
        Vec3f edge1 = v1 - v0;
        Vec3f edge2 = v2 - v0;

        /*Vec3f n = crossProduct(edge1, edge2);
        float NdotRayDirection = dotProduct(n, dir);
        float d = dotProduct(n, v0); 
        float t = (dotProduct(n, orig) + d) / NdotRayDirection; 
        if (t < 0) return false;*/

        Vec3f pvec = crossProduct(dir, edge2);
        float det = dotProduct(edge1, pvec);
        if (det == 0 || det < 0) return false;

        Vec3f tvec = orig - v0;
        uv.x = dotProduct(tvec, pvec);
        if (uv.x < 0 || uv.x > det) return false;

        Vec3f qvec = crossProduct(tvec, edge1);
        uv.y = dotProduct(dir, qvec);
        if (uv.y < 0 || uv.x + uv.y > det) return false;

        float invDet = 1 / det;

        tnear = dotProduct(edge2, qvec) * invDet;
        uv.x *= invDet;
        uv.y *= invDet;


        /*std::cout << orig << std::endl;
        std::cout << dir << std::endl;
        std::cout << v0 << std::endl;
        std::cout << v1 << std::endl;
        std::cout << v2 << std::endl;
        std::cout << det << std::endl;*/

        return true;   
    }

    void getSurfaceProperties(const Vec3f &P, const Vec3f &I, const uint32_t &index, const Vec2f &uv, Vec3f &N, Vec2f &st) const
    {
        Vec3f edge1 = normalize(v1 - v0); 
        Vec3f edge2 = normalize(v2 - v1); 
        N = normalize(crossProduct(edge1, edge2)); 
        // add st if you want the tile texture
    }

    Bounds3f objectBounds() const {
        return Bounds3f(Vec3f(std::min(std::min(v0.x, v1.x), v2.x),
                              std::min(std::min(v0.y, v1.y), v2.y),
                              std::min(std::min(v0.z, v1.z), v2.z)),
                        Vec3f(std::max(std::max(v0.x, v1.x), v2.x),
                              std::max(std::max(v0.y, v1.y), v2.y),
                              std::max(std::max(v0.z, v1.z), v2.z)));
    }

    Vec3f v0, v1, v2;
};

 
class MeshTriangle : public Object 
{ 
public: 
    MeshTriangle( 
        const Vec3f *verts, 
        const uint32_t *vertsIndex, 
        const uint32_t &numTris, 
        const Vec2f *st) 
    { 
        uint32_t maxIndex = 0; 
        for (uint32_t i = 0; i < numTris * 3; ++i) 
            if (vertsIndex[i] > maxIndex) maxIndex = vertsIndex[i]; 
        maxIndex += 1; 
        vertices = std::unique_ptr<Vec3f[]>(new Vec3f[maxIndex]); 
        memcpy(vertices.get(), verts, sizeof(Vec3f) * maxIndex); 
        vertexIndex = std::unique_ptr<uint32_t[]>(new uint32_t[numTris * 3]); 
        memcpy(vertexIndex.get(), vertsIndex, sizeof(uint32_t) * numTris * 3); 
        numTriangles = numTris; 
        stCoordinates = std::unique_ptr<Vec2f[]>(new Vec2f[maxIndex]); 
        memcpy(stCoordinates.get(), st, sizeof(Vec2f) * maxIndex); 
        isMesh = true;
    } 
 
    bool intersect(const Vec3f &orig, const Vec3f &dir, float &tnear, uint32_t &index, Vec2f &uv) const 
    { 
        bool intersect = false; 
        for (uint32_t k = 0; k < numTriangles; ++k) { 
            const Vec3f & v0 = vertices[vertexIndex[k * 3]]; 
            const Vec3f & v1 = vertices[vertexIndex[k * 3 + 1]]; 
            const Vec3f & v2 = vertices[vertexIndex[k * 3 + 2]]; 
            // std::cout << v0 << " +  " << v1 << ", + " << v2 << std::endl;
            float t, u, v; 
            if (rayTriangleIntersect(v0, v1, v2, orig, dir, t, u, v) && t < tnear) { 
                tnear = t; 
                uv.x = u; 
                uv.y = v; 
                index = k; 
                intersect |= true; 
            } 
        } 
 
        return intersect; 
    } 
 
    void getSurfaceProperties(const Vec3f &P, const Vec3f &I, const uint32_t &index, const Vec2f &uv, Vec3f &N, Vec2f &st) const 
    { 
        const Vec3f &v0 = vertices[vertexIndex[index * 3]]; 
        const Vec3f &v1 = vertices[vertexIndex[index * 3 + 1]]; 
        const Vec3f &v2 = vertices[vertexIndex[index * 3 + 2]]; 
        Vec3f e0 = normalize(v1 - v0); 
        Vec3f e1 = normalize(v2 - v1); 
        N = normalize(crossProduct(e0, e1)); 
        const Vec2f &st0 = stCoordinates[vertexIndex[index * 3]]; 
        const Vec2f &st1 = stCoordinates[vertexIndex[index * 3 + 1]]; 
        const Vec2f &st2 = stCoordinates[vertexIndex[index * 3 + 2]]; 
        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y; 
    } 
 
    Vec3f evalDiffuseColor(const Vec2f &st) const 
    { 
        float scale = 5; 
        float pattern = (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5); 
        return mix(Vec3f(0.815, 0.235, 0.031), Vec3f(0.937, 0.937, 0.231), pattern); 
    } 

    Bounds3f objectBounds() const {
        return Bounds3f();
    }
 
    std::unique_ptr<Vec3f[]> vertices; 
    uint32_t numTriangles; 
    std::unique_ptr<uint32_t[]> vertexIndex; 
    std::unique_ptr<Vec2f[]> stCoordinates; 
};


#endif
