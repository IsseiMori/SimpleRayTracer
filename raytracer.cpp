#include <cstdio> 
#include <cstdlib> 
#include <memory> 
#include <vector> 
#include <utility> 
#include <cstdint> 
#include <iostream> 
#include <fstream> 
#include <cmath> 

#ifndef RAYTRACERCPP_INCLUDE
#define RAYTRACERCPP_INCLUDE

#include "util.h"
#include "bvh.h"

 
// Compute reflection direction 

Vec3f reflect(const Vec3f &I, const Vec3f &N) 
{ 
    return I - 2 * dotProduct(I, N) * N; 
} 
 
/*
Compute refraction direction using Snell's law
We need to handle with care the two possible situations:
   - When the ray is inside the object
   - When the ray is outside.
If the ray is outside, you need to make cosi positive cosi = -N.I
If the ray is inside, you need to invert the refractive indices and negate the normal N 
*/
Vec3f refract(const Vec3f &I, const Vec3f &N, const float &ior) 
{ 
    float cosi = clamp(-1, 1, dotProduct(I, N)); 
    float etai = 1, etat = ior; 
    Vec3f n = N; 
    if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; } 
    float eta = etai / etat; 
    float k = 1 - eta * eta * (1 - cosi * cosi); 
    return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n; 
} 
/*
Compute Fresnel equation
\param I is the incident view direction
\param N is the normal at the intersection point
\param ior is the mateural refractive index
\param[out] kr is the amount of light reflected 
*/

void fresnel(const Vec3f &I, const Vec3f &N, const float &ior, float &kr) 
{ 
    float cosi = clamp(-1, 1, dotProduct(I, N)); 
    float etai = 1, etat = ior; 
    if (cosi > 0) {  std::swap(etai, etat); } 
    // Compute sini using Snell's law
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi)); 
    // Total internal reflection
    if (sint >= 1) { 
        kr = 1; 
    } 
    else { 
        float cost = sqrtf(std::max(0.f, 1 - sint * sint)); 
        cosi = fabsf(cosi); 
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost)); 
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost)); 
        kr = (Rs * Rs + Rp * Rp) / 2; 
    } 
    // As a consequence of the conservation of energy, transmittance is given by:
    // kt = 1 - kr;
} 

/*
Returns true if the ray intersects an object, false otherwise.
\param orig is the ray origin
\param dir is the ray direction
\param objects is the list of objects the scene contains
\param[out] tNear contains the distance to the cloesest intersected object.
\param[out] index stores the index of the intersect triangle if the interesected object is a mesh.
\param[out] uv stores the u and v barycentric coordinates of the intersected point
\param[out] *hitObject stores the pointer to the intersected object (used to retrieve material information, etc.)
\param isShadowRay is it a shadow ray. We can return from the function sooner as soon as we have found a hit. 
*/

bool trace( 
    const Vec3f &orig, const Vec3f &dir, 
    const std::vector<std::shared_ptr<Object>> &objects, 
    float &tNear, uint32_t &index, Vec2f &uv, Object **hitObject) 
{
    *hitObject = nullptr;

    for (uint32_t k = 0; k < objects.size(); ++k) { 
        float tNearK = kInfinity; 
        uint32_t indexK; 
        Vec2f uvK; 
        if (objects[k]->intersect(orig, dir, tNearK, indexK, uvK) && tNearK < tNear) { 
            *hitObject = objects[k].get(); 
            tNear = tNearK; 
            index = indexK; 
            uv = uvK; 
        } 
    } 
 
    return (*hitObject != nullptr); 
} 

bool traceBVH(
    const BVHAccel &bvh,
    const Vec3f &orig, const Vec3f &dir, 
    const std::vector<std::shared_ptr<Object>> &objects, 
    float &tNear, uint32_t &index, Vec2f &uv, Object **hitObject) 
{
    *hitObject = nullptr;

    if (bvh.intersect(orig, dir, tNear, index, uv)) {
        *hitObject = bvh.orderedObjects[index].get();
    }

 
    return (*hitObject != nullptr); 
} 
 
/*
Implementation of the Whitted-syle light transport algorithm (E [S*] (D|G) L)
This function is the function that compute the color at the intersection point of a ray defined by a position and a direction. Note that thus function is recursive (it calls itself).
If the material of the intersected object is either reflective or reflective and refractive, then we compute the reflection/refracton direction and cast two new rays into the scene by calling the castRay() function recursively. When the surface is transparent, we mix the reflection and refraction color using the result of the fresnel equations (it computes the amount of reflection and refractin depending on the surface normal, incident view direction and surface refractive index).
If the surface is duffuse/glossy we use the Phong illumation model to compute the color at the intersection point. 
*/

Vec3f castRay( 
    const BVHAccel &bvh,
    const Vec3f &orig, const Vec3f &dir,
    const std::vector<std::shared_ptr<Object>> &objects,
    const std::vector<std::unique_ptr<Light>> &lights,
    const Options &options, 
    uint32_t depth, 
    bool test = false) 
{ 
    if (depth > options.maxDepth) { 
        return options.backgroundColor; 
    } 
 
    Vec3f hitColor = options.backgroundColor; 
    float tnear = kInfinity; 
    Vec2f uv; 
    uint32_t index = 0; 
    Object *hitObject = nullptr; 
    if (traceBVH(bvh, orig, dir, objects, tnear, index, uv, &hitObject)) { 
        Vec3f hitPoint = orig + dir * tnear; 
        Vec3f N; // normal 
        Vec2f st; // st coordinates 
        hitObject->getSurfaceProperties(hitPoint, dir, index, uv, N, st); 
        Vec3f tmp = hitPoint; 
        switch (hitObject->materialType) { 
            case REFLECTION_AND_REFRACTION: 
            { 
                Vec3f reflectionDirection = normalize(reflect(dir, N)); 
                Vec3f refractionDirection = normalize(refract(dir, N, hitObject->ior)); 
                Vec3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ? 
                    hitPoint - N * options.bias : 
                    hitPoint + N * options.bias; 
                Vec3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ? 
                    hitPoint - N * options.bias : 
                    hitPoint + N * options.bias; 
                Vec3f reflectionColor = castRay(bvh, reflectionRayOrig, reflectionDirection, objects, lights, options, depth + 1, 1); 
                Vec3f refractionColor = castRay(bvh, refractionRayOrig, refractionDirection, objects, lights, options, depth + 1, 1); 
                float kr; 
                fresnel(dir, N, hitObject->ior, kr); 
                hitColor = reflectionColor * kr + refractionColor * (1 - kr); 
                break; 
            } 
            case REFLECTION: 
            { 
                float kr; 
                fresnel(dir, N, hitObject->ior, kr); 
                Vec3f reflectionDirection = reflect(dir, N); 
                Vec3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ? 
                    hitPoint + N * options.bias : 
                    hitPoint - N * options.bias; 
                hitColor = castRay(bvh, reflectionRayOrig, reflectionDirection, objects, lights, options, depth + 1) * kr; 
                break; 
            } 
            default: 
            { 
// We use the Phong illumation model int the default case. The phong model is composed of a diffuse and a specular reflection component. 

                Vec3f lightAmt = 0, specularColor = 0; 
                Vec3f shadowPointOrig = (dotProduct(dir, N) < 0) ? 
                    hitPoint + N * options.bias : 
                    hitPoint - N * options.bias; 
// Loop over all lights in the scene and sum their contribution up We also apply the lambert cosine law here though we haven't explained yet what this means. 

                for (uint32_t i = 0; i < lights.size(); ++i) { 
                    Vec3f lightDir = lights[i]->position - hitPoint; 
                    // square of the distance between hitPoint and the light
                    float lightDistance2 = dotProduct(lightDir, lightDir); 
                    lightDir = normalize(lightDir); 
                    float LdotN = std::max(0.f, dotProduct(lightDir, N)); 
                    Object *shadowHitObject = nullptr; 
                    float tNearShadow = kInfinity; 
                    // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                    bool inShadow = trace(shadowPointOrig, lightDir, objects, tNearShadow, index, uv, &shadowHitObject) && 
                        tNearShadow * tNearShadow < lightDistance2; 
                    lightAmt += (1 - inShadow) * lights[i]->intensity * LdotN; 
                    Vec3f reflectionDirection = reflect(-lightDir, N); 
                    specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, dir)), hitObject->specularExponent) * lights[i]->intensity; 
                } 
                hitColor = lightAmt * hitObject->evalDiffuseColor(st) * hitObject->Kd + specularColor * hitObject->Ks; 
                break; 
            } 
        } 
    } 
 
    return hitColor; 
} 
 
// The main render function. This where we iterate over all pixels in the image, generate primary rays and cast these rays into the scene. The content of the framebuffer is saved to a file. 

void render( 
    const Options &options, 
    const std::vector<std::shared_ptr<Object>> &objects, 
    const std::vector<std::unique_ptr<Light>> &lights) 
{ 
    Vec3f *framebuffer = new Vec3f[options.width * options.height]; 
    Vec3f *pix = framebuffer; 
    float scale = tan(deg2rad(options.fov * 0.5)); 
    float imageAspectRatio = options.width / (float)options.height; 
    Vec3f orig(0); 

    // BVH
    BVHAccel bvh = BVHAccel(objects);

    for (uint32_t j = 0; j < options.height; ++j) { 
        for (uint32_t i = 0; i < options.width; ++i) { 
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)options.width - 1) * imageAspectRatio * scale; 
            float y = (1 - 2 * (j + 0.5) / (float)options.height) * scale; 
            Vec3f dir = normalize(Vec3f(x, y, -1)); 
            *(pix++) = castRay(bvh, orig, dir, objects, lights, options, 0);
            // if (j == 356 && i = 545) std::cout << 
        } 
    } 
 
    // save framebuffer to file
    std::ofstream ofs; 
    ofs.open("./out.ppm"); 
    ofs << "P6\n" << options.width << " " << options.height << "\n255\n"; 
    for (uint32_t i = 0; i < options.height * options.width; ++i) { 
        char r = (char)(255 * clamp(0, 1, framebuffer[i].x)); 
        char g = (char)(255 * clamp(0, 1, framebuffer[i].y)); 
        char b = (char)(255 * clamp(0, 1, framebuffer[i].z)); 
        ofs << r << g << b; 
    } 
 
    ofs.close(); 
 
    delete [] framebuffer; 
} 
 
// In the main function of the program, we create the scene (create objects and lights) as well as set the options for the render (image widht and height, maximum recursion depth, field-of-view, etc.). We then call the render function(). 

int main(int argc, char **argv) 
{ 
    // creating the scene (adding objects and lights)
    std::vector<std::shared_ptr<Object>> objects; 
    std::vector<std::unique_ptr<Light>> lights; 
 
    Sphere *sph1 = new Sphere(Vec3f(-1, 0, -12), 2); 
    sph1->materialType = DIFFUSE_AND_GLOSSY; 
    sph1->diffuseColor = Vec3f(0.6, 0.7, 0.8); 

    Sphere *sph2 = new Sphere(Vec3f(0.5, -0.5, -8), 1.5); 
    sph2->ior = 1.5; 
    sph2->materialType = REFLECTION_AND_REFRACTION; 

    Sphere *sph3 = new Sphere(Vec3f(6, 0, -12), 2); 
    sph3->materialType = DIFFUSE_AND_GLOSSY; 
    sph3->diffuseColor = Vec3f(0.6, 0.7, 0.8); 
 
    objects.push_back(std::unique_ptr<Sphere>(sph1)); 
    objects.push_back(std::unique_ptr<Sphere>(sph2)); 
    objects.push_back(std::unique_ptr<Sphere>(sph3));

    // Triangle Mesh Counte Clock wise

    // Floor
    Triangle *tri1 = new Triangle(Vec3f(-5,-3,-5), Vec3f(5,-3,-15), Vec3f(-5,-3,-15));
    tri1->materialType = DIFFUSE_AND_GLOSSY;
    tri1->diffuseColor = Vec3f(1.0, 0, 0);

    Triangle *tri2 = new Triangle(Vec3f(-5,-3,-5), Vec3f(5,-3,-5), Vec3f(5,-3,-15));
    tri2->materialType = DIFFUSE_AND_GLOSSY;
    tri2->diffuseColor = Vec3f(1.0, 0, 0);

    objects.push_back(std::unique_ptr<Triangle>(tri1));
    objects.push_back(std::unique_ptr<Triangle>(tri2));

    // Wall
    Triangle *tri3 = new Triangle(Vec3f(-5,-3,-5), Vec3f(-5,-3,-15), Vec3f(-5,10,-15));
    tri3->materialType = DIFFUSE_AND_GLOSSY;
    tri3->diffuseColor = Vec3f(0, 1.0, 0);

    Triangle *tri4 = new Triangle(Vec3f(-5,-3,-5), Vec3f(-5,10,-15), Vec3f(-5,10,-5));
    tri4->materialType = DIFFUSE_AND_GLOSSY;
    tri4->diffuseColor = Vec3f(0, 1.0, 0);

    objects.push_back(std::unique_ptr<Triangle>(tri3));
    objects.push_back(std::unique_ptr<Triangle>(tri4));

    Triangle *tri = new Triangle(Vec3f({2,-3,-10}), Vec3f(2,-3,-16), Vec3f(-2,-3,-16));
    tri->materialType = DIFFUSE_AND_GLOSSY;
    tri->diffuseColor = Vec3f(1.0, 0, 0);

    // objects.push_back(std::unique_ptr<Triangle>(tri));
    /*
    Vec3f verts[4] = {{-2,-3,-10}, {2,-3,-10}, {2,-3,-16}, {-2,-3,-16}}; 
    // uint32_t vertIndex[6] = {0, 1, 3, 1, 2, 3}; 
    uint32_t vertIndex[6] = {1, 2, 3, 1, 2, 3}; 
    Vec2f st[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}}; 
    MeshTriangle *mesh = new MeshTriangle(verts, vertIndex, 2, st); 
    mesh->materialType = DIFFUSE_AND_GLOSSY; 
 
    objects.push_back(std::unique_ptr<MeshTriangle>(mesh)); 
    */
    
 
    lights.push_back(std::unique_ptr<Light>(new Light(Vec3f(-20, 70, 20), 0.5))); 
    lights.push_back(std::unique_ptr<Light>(new Light(Vec3f(30, 50, -12), 1))); 

    // 5, -3, 6 +  5, -3, -16, + -5, -3, -16
 
    // setting up options
    Options options; 
    options.width = 640; 
    options.height = 480; 
    options.fov = 90; 
    options.backgroundColor = Vec3f(0.235294, 0.67451, 0.843137); 
    options.maxDepth = 2; 
    options.bias = 0.00001; 
 
    // finally, render
    render(options, objects, lights); 
 
    return 0; 
}
#endif