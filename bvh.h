#include <cstdio> 
#include <cstdlib> 
#include <memory> 
#include <vector> 
#include <utility> 
#include <cstdint> 
#include <iostream> 
#include <fstream> 
#include <cmath> 

#ifndef BVHCPP_INCLUDE
#define BVHCPP_INCLUDE

#include "util.h"

// Need to create primitive wrapper???
// Can just put bounds and material in each shape

/*
** Compute bounding for each object and store in an array
** Split space and build a binary tree on objects
** Convert the tree into pointerless representation
*/


struct BVHObjectInfo {
    BVHObjectInfo(size_t _objectNumber, const Bounds3f &_bounds)
        : objectNumber(_objectNumber), bounds(_bounds),
          centroid(.5f * bounds.pMin + .5f * bounds.pMax) {}
    size_t objectNumber; // index in objects array
    Bounds3f bounds;
    Vec3f centroid;
};

struct BVHBuildNode {
    Bounds3f bounds;
    BVHBuildNode *children[2];
    int splitAxis, firstObjOffset, nObjects;

    void InitLeaf(int first, int n, const Bounds3f &b) {
        firstObjOffset = first;
        nObjects = n;
        bounds = b;
        children[0] = children[1] = nullptr;
    }

    // Once Node is created by InitLeaf, call this to set child nodes
    void InitInterior(int axis, BVHBuildNode *c0, BVHBuildNode *c1) {
        children[0] = c0;
        children[1] = c1;
        bounds = Union(c0->bounds, c1->bounds);
        splitAxis = axis;
        nObjects = 0;
    }

};

class BVHAccel {
public:
    BVHAccel(std::vector<Object> &o) {}

    // Build a BVH binary tree
    BVHBuildNode *recursiveBuild(std::vector<BVHObjectInfo> &objectInfo, int start,
                                 int end, int *totalNodes, 
                                 std::vector<std::shared_ptr<Object>> &orderedObjects) {
        BVHBuildNode *node;
        (*totalNodes)++;

        // Compute bounds for all objects in BVH node
        Bounds3f bounds;
        for (int i = start; i < end; ++i) 
            bounds = Union(bounds, objectInfo[i].bounds);

        int nObjects = end - start;

        // 
        if (nObjects == 1) {
            int firstObjOffset = orderedObjects.size();
            for (int i = start; i < end; ++i) {
                int objNum = objectInfo[i].objectNumber;
                orderedObjects.push_back(objects[objNum]);
            }
            node->InitLeaf(firstObjOffset, nObjects, bounds);
            return node;
        } else {

            // Find the axis with the most extent between centroids
            Bounds3f centroidBounds;
            for (int i = start; i < end; ++i) {
                centroidBounds = Union(centroidBounds, objectInfo[i].centroid);
            }
            int dim = centroidBounds.MaximumExtent();

            int mid = (start - end) / 2;

            // if all objects are at one point, push all to the array
            if (centroidBounds.pMax[dim] == centroidBounds.pMin[dim]) {
                int firstObjOffset = orderedObjects.size();
                for (int i = start; i < end; ++i) {
                    int objNum = objectInfo[i].objectNumber;
                    orderedObjects.push_back(objects[objNum]);
                }
                node->InitLeaf(firstObjOffset, nObjects, bounds);
                return node;
            } else {
                /* THIS IS MID POINT METHOD
                // find the ptr to the mid element
                float pmid = (centroidBounds.pMin[dim] + centroidBounds.pMax[dim]) / 2;
                BVHObjectInfo *midPtr = 
                    std::partition(&objectInfo[start], &objectInfo[end-1]+1,
                        [dim, pmid](const BVHObjectInfo &pi) {
                            return pi.centroid[dim] < pmid;
                        });
                mid = midPtr - &objectInfo[0];
                if (mid != start && mid != end)
                    break;
                */
                // THIS IS EQUAL
                mid = (start - end) / 2;
                // order objectInfo so that mid is the middle element if fully sorted,
                // everything before mid is less than mid, after mid is more than mid
                std::nth_element(&objectInfo[start], &objectInfo[mid], 
                                 &objectInfo[end-1]+1,
                                 [dim](const BVHObjectInfo &a, const BVHObjectInfo &b) {
                                    return a.centroid[dim] < b.centroid[dim];
                                 });
                node->InitInterior(dim, 
                                   recursiveBuild(objectInfo, start, mid,
                                                  totalNodes, orderedObjects),
                                   recursiveBuild(objectInfo, mid, end,
                                                  totalNodes, orderedObjects));

            }
        }

        return node;
    }

    bool intersect(const Vec3f &orig, const Vec3f &dir, float &tnear, uint32_t &index, Vec2f &uv) {
        bool hit = false;
        int dirIsNeg[] = {dir.x < 0, dir.y < 0, dir.z < 0};
    }

    std::vector<std::shared_ptr<Object>> objects;
};

#endif