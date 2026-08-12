#include "hitscanprocessing.h"
#include <iostream>

#include <algorithm>
#include <array>
#include <numeric>
#include <limits>
#include <cmath>

//#define TINYGLTF_IMPLEMENTATION
//#define STB_IMAGE_IMPLEMENTATION
//#define STB_IMAGE_WRITE_IMPLEMENTATION
//#if defined( WIN32 )
//#pragma warning( push )
//#pragma warning( disable : 4267 )
//#endif
//#include <support/tinygltf/tiny_gltf.h>
//#if defined( WIN32 )
//#pragma warning( pop )
//#endif

namespace sutil {
namespace hitscan {

namespace
{

bool raycastBVHNode(
    TriangleMesh&,
    int,
    float3,
    float3,
    float&,
    float3&,
    float3&);
bool intersectTriangle(
    const Triangle&,
    const float3&,
    const float3&,
    float&,
    float3&);
bool intersectRayAabb(
    const Aabb&,
    const float3&,
    const float3&,
    float,
    float&,
    float&);
Aabb computeTriangleRangeAabb(
    const TriangleMesh&,
    int,
    int);
int buildBVHNode(
    TriangleMesh&,
    const std::vector<float3>&,
    int,
    int);


constexpr int BVH_LEAF_TRIANGLE_COUNT = 32;

size_t visitedNodes = 0;


bool raycastBVHNode(
    TriangleMesh& tm,
    int nodeIndex,
    float3 rayOrigin,
    float3 rayDir,
    float& closestT,
    float3& hitPoint,
    float3& hitNormal)
{
    bool foundHit = false;

    std::array<int, 64> stack;

    int stackSize = 0;

    stack[stackSize++] = nodeIndex;

    static size_t testedTriangles = 0;

    while(stackSize > 0)
    {
        const int nodeIndex = stack[--stackSize];

        visitedNodes++;

        const BVHNode& node = tm.bvh[nodeIndex];


        float nodeTMin;
        float nodeTMax;

        if(!intersectRayAabb(node.bounds, rayOrigin, rayDir, closestT, nodeTMin, nodeTMax))
        {
            continue;
        }

        
        if(node.isLeaf())
        {
            for(int i = 0; i < node.triangleCount; ++i)
            {
                testedTriangles++;
                const int triangleIndex = tm.triangleIndices[node.firstTriangle + i];
                const Triangle& triangle = tm.triangles[triangleIndex];
                float t;
                float3 normal;

                if(intersectTriangle(triangle, rayOrigin, rayDir, t, normal))
                {
                    if(t < closestT)
                    {
                        closestT = t;
                        hitPoint = rayOrigin + t * rayDir;
                        hitNormal = normal;
                        foundHit = true;
                    }
                }
            }

            continue;
        }


        const int left = node.left;
        const int right = node.right;

        float leftTMin, leftTMax;
        float rightTMin, rightTMax;

        const bool hitLeft = intersectRayAabb(tm.bvh[left].bounds, rayOrigin, rayDir, closestT, leftTMin, leftTMax);
        const bool hitRight = intersectRayAabb(tm.bvh[right].bounds, rayOrigin, rayDir, closestT, rightTMin, rightTMax);

        if(hitLeft && hitRight)
        {
            if(leftTMin < rightTMin)
            {
                stack[stackSize++] = right;
                stack[stackSize++] = left;
            }
            else
            {
                stack[stackSize++] = left;
                stack[stackSize++] = right;
            }
        }
        else if(hitLeft)
        {
            stack[stackSize++] = left;
        }
        else if(hitRight)
        {
            stack[stackSize++] = right;
        }
    }

    testedTriangles = 0;

    return foundHit;
}

bool intersectTriangle(
    const Triangle& triangle,
    const float3& rayOrigin,
    const float3& rayDir,
    float& t,
    float3& normal)
{
    constexpr float EPSILON = 1e-7f;

    const float3 edge1 = triangle.p1 - triangle.p0;
    const float3 edge2 = triangle.p2 - triangle.p0;
    const float3 pvec = cross(rayDir, edge2);
    const float determinant = dot(edge1, pvec);

    if(std::fabs(determinant) < EPSILON)
        return false;

    const float inverseDeterminant = 1.0f / determinant;
    const float3 tvec = rayOrigin - triangle.p0;
    const float u = dot(tvec, pvec) * inverseDeterminant;

    if(u < 0.0f || u > 1.0f)
        return false;

    const float3 qvec = cross(tvec, edge1);
    const float v = dot(rayDir, qvec) * inverseDeterminant;

    if(v < 0.0f || u + v > 1.0f)
        return false;

    t = dot(edge2, qvec) * inverseDeterminant;

    if(t < 0.0f)
        return false;

    normal = normalize(cross(edge1, edge2));

    return true;
}

inline float component(const float3& v, int axis)
{
    if(axis==0) return v.x;
    if(axis==1) return v.y;
    return v.z;
}

bool intersectRayAabb(
    const Aabb& box,
    const float3& rayOrigin,
    const float3& rayDir,
    float maxT,
    float& tMinOut,
    float& tMaxOut)
{
    float tMin = 0.0f;
    float tMax = maxT;

    for(int axis = 0; axis < 3; ++axis)
    {
        const float origin = component(rayOrigin, axis);

        const float direction = component(rayDir, axis);

        const float minValue = component(box.m_min, axis);

        const float maxValue = component(box.m_max, axis);

        if(std::fabs(direction) < 1e-8f)
        {
            if(origin < minValue || origin > maxValue)
                return false;

            continue;
        }

        const float invDirection = 1.0f / direction;
        float t0 = (minValue - origin) * invDirection;
        float t1 = (maxValue - origin) * invDirection;

        if(t0 > t1)
            std::swap(t0, t1);

        tMin = std::fmax(tMin, t0);
        tMax = std::fmin(tMax, t1);

        if(tMin > tMax)
            return false;
    }

    tMinOut = tMin;
    tMaxOut = tMax;

    return true;
}

Aabb computeTriangleRangeAabb(const TriangleMesh& tm, int begin, int end)
{
    Aabb bounds;
    bounds.invalidate();

    for(int i = begin; i < end; ++i)
    {
        const int triangleIndex = tm.triangleIndices[i];
        const Triangle& triangle = tm.triangles[triangleIndex];

        bounds.include(triangle.p0);
        bounds.include(triangle.p1);
        bounds.include(triangle.p2);
    }

    return bounds;
}

int buildBVHNode(TriangleMesh& tm, const std::vector<float3>& centroids, int begin, int end)
{
    const int nodeIndex = static_cast<int>(tm.bvh.size());

    tm.bvh.emplace_back();

    const int triangleCount = end - begin;

    if(triangleCount <= BVH_LEAF_TRIANGLE_COUNT)
    {
        tm.bvh[nodeIndex].bounds = computeTriangleRangeAabb(tm, begin, end);
        tm.bvh[nodeIndex].firstTriangle = begin;
        tm.bvh[nodeIndex].triangleCount = triangleCount;

        return nodeIndex;
    }

    Aabb centroidBounds;
    centroidBounds.invalidate();

    for(int i = begin; i < end; ++i)
    {
        centroidBounds.include(centroids[tm.triangleIndices[i]]);
    }

    const int axis = centroidBounds.longestAxis();

    const int mid = begin + triangleCount / 2;

    std::nth_element(
        tm.triangleIndices.begin() + begin,
        tm.triangleIndices.begin() + mid,
        tm.triangleIndices.begin() + end,

        [&centroids, axis](int a, int b)
        {
            return component(centroids[a], axis) < component(centroids[b], axis);
        }
    );

    const int leftChild = buildBVHNode(tm, centroids, begin, mid);

    const int rightChild = buildBVHNode(tm, centroids, mid, end);

    Aabb bounds = tm.bvh[leftChild].bounds;
    bounds.include(tm.bvh[rightChild].bounds);

    tm.bvh[nodeIndex].bounds = bounds;
    tm.bvh[nodeIndex].left = leftChild;
    tm.bvh[nodeIndex].right = rightChild;

    return nodeIndex;
}

}


void buildBVH(TriangleMesh& tm)
{
    tm.bvh.clear();

    const size_t triangleCount = tm.triangles.size();

    if(triangleCount == 0)
        return;

    tm.triangleIndices.resize(triangleCount);

    std::iota(tm.triangleIndices.begin(), tm.triangleIndices.end(), 0);

    std::vector<float3> centroids(triangleCount);

    for(size_t i = 0; i < triangleCount; ++i)
    {
        const Triangle& t = tm.triangles[i];

        centroids[i] = (t.p0 + t.p1 + t.p2) / 3.0f;
    }

    const size_t leafCount = (triangleCount + BVH_LEAF_TRIANGLE_COUNT - 1) / BVH_LEAF_TRIANGLE_COUNT;

    tm.bvh.reserve(leafCount * 2);

    buildBVHNode(tm, centroids, 0, static_cast<int>(triangleCount));
}

bool raycastBVH(
    TriangleMesh& tm,
    float3 rayOrigin,
    float3 rayDir,
    float& closestT,
    float3& hitPoint,
    float3& hitNormal)
{
    if (tm.bvh.empty())
        return false;

    visitedNodes = 0;

    bool result = raycastBVHNode(tm, 0, rayOrigin, rayDir, closestT, hitPoint, hitNormal);

    return result;
}

bool raycastMesh(
    TriangleMesh& tm,
    float3 rayStart,
    float3 rayDir,
    float3& hitPoint,
    float3& hitNormal)
{
    if(tm.bvh.empty())
        return false;

    float worldTMin;
    float worldTMax;

    if(!intersectRayAabb(tm.worldAabb, rayStart, rayDir, 1e30f, worldTMin, worldTMax))
    {
        return false;
    }

    const Matrix4x4 inverseTransform = tm.transform.inverse();

    const float3 objectRayStart = make_float3(inverseTransform * make_float4(rayStart, 1.0f));
    const float3 objectRayDir = normalize(make_float3(inverseTransform * make_float4(rayDir, 0.0f)));

    float closestT = 1e30f;

    float3 objectHit;
    float3 objectNormal;

    if(!raycastBVH(tm, objectRayStart, objectRayDir, closestT, objectHit, objectNormal))
    {
        return false;
    }

    hitPoint = make_float3(tm.transform * make_float4(objectHit, 1.0f));
    hitNormal = normalize(make_float3(tm.transform * make_float4(objectNormal, 0.0f)));

    return true;
}

}
}

/////////////////////////////////////////////////////////////
// Performing hitscans
/////////////////////////////////////////////////////////////

const bool sutil::hitscan::isPointWithinMesh(sutil::hitscan::TriangleMesh& tm, float3 worldPoint)
{
  float3 objectPos = make_float3(tm.transform.inverse() * make_float4(worldPoint));
  float3 rayStartPos = objectPos;
  rayStartPos.x = tm.objectAabb.m_min.x - 1.0f;
  float3 rayDir = normalize(objectPos - rayStartPos);
  float d = 0.12f;
  unsigned int intersectionCount = 0;
  for(Triangle triangle : tm.triangles )
  {
    // Basically just return true if near a vertex, forming spheres around the vertices for testing purposes
    //if(length(triangle.p0 - objectPos) < d ||
    //   length(triangle.p1 - objectPos) < d ||
    //   length(triangle.p2 - objectPos) < d )
    //   return true;

    //// Test if the triangle intersects the ray
    float3 planeNormal = normalize(cross((triangle.p1-triangle.p0), (triangle.p2-triangle.p0)));

    float denominator = dot(planeNormal, rayDir);
    if(denominator == 0)
      continue; // Don't test against this triangle if it's parallel to the ray (infinite intersections)

    float distanceToPlaneAlongRay = dot((triangle.p0 - rayStartPos), planeNormal) / denominator;

    if(distanceToPlaneAlongRay == 0)
      continue; // The ray sits in the plane, so don't count it.

    float3 hitLocation = rayStartPos + distanceToPlaneAlongRay * rayDir;

    // Skip this ray if it intersected behind the raycast direction
    // or if the intersection location was past where the target location is, in object-space.
    if(distanceToPlaneAlongRay < 0 || hitLocation.x > objectPos.x)
      continue;
    
    //// Make sure that the intersection is less than or equal to limit
    float3 edge, fromEdgeStart, crossProd;

    //// First edge
    edge = triangle.p1 - triangle.p0;
    fromEdgeStart = hitLocation - triangle.p0;
    crossProd = cross(edge, fromEdgeStart);
    if(dot(planeNormal, crossProd) < 0)
      continue;

    //// Second edge
    edge = triangle.p2 - triangle.p1;
    fromEdgeStart = hitLocation - triangle.p1;
    crossProd = cross(edge, fromEdgeStart);
    if(dot(planeNormal, crossProd) < 0)
      continue;

    //// Third edge
    edge = triangle.p0 - triangle.p2;
    fromEdgeStart = hitLocation - triangle.p2;
    crossProd = cross(edge, fromEdgeStart);
    if(dot(planeNormal, crossProd) < 0)
      continue;
    
    // Finally, if it's a good hit and inside the triangle, then add to the intersection count
    intersectionCount++;
  }
  return intersectionCount%2 == 1;
}

void sutil::hitscan::calculateObjectAabb(sutil::hitscan::TriangleMesh& tm)
{
  float3 minPos = tm.triangles[0].p0;
  float3 maxPos = tm.triangles[0].p0;

  for(auto triangle : tm.triangles)
  {
    minPos = fminf(minPos, triangle.p0);
    minPos = fminf(minPos, triangle.p1);
    minPos = fminf(minPos, triangle.p2);
    maxPos = fmaxf(maxPos, triangle.p0);
    maxPos = fmaxf(maxPos, triangle.p1);
    maxPos = fmaxf(maxPos, triangle.p2);
  }

  tm.objectAabb = Aabb(minPos, maxPos);
}

void sutil::hitscan::calculateWorldAabbUsingTransformAndObjectAabb(sutil::hitscan::TriangleMesh& tm)
{
  tm.worldAabb = tm.objectAabb;
  tm.worldAabb.transform(tm.transform);
}



/////////////////////////////////////////////////////////////
// Loading gltf models as triangle meshes
/////////////////////////////////////////////////////////////

void sutil::hitscan::populateTriangleMesh(sutil::hitscan::TriangleMesh& tm, const tinygltf::Mesh& mesh, const tinygltf::Model& model)
{
  for( auto& primitive : mesh.primitives )
  {
      if( primitive.mode != TINYGLTF_MODE_TRIANGLES ) // Ignore non-triangle meshes
      {
          std::cerr << "\tNon-triangle primitive: skipping\n";
          continue;
      }

      // Switch based on how the indicies are stored
      const tinygltf::Accessor& indexAccessor = model.accessors[primitive.indices];
      switch(indexAccessor.componentType)
      {
        default:
        case TINYGLTF_COMPONENT_TYPE_BYTE:
          getTriangles<int8_t>(tm, model, primitive);
          break;
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
          getTriangles<uint8_t>(tm, model, primitive);
          break;
        case TINYGLTF_COMPONENT_TYPE_SHORT:
          getTriangles<int16_t>(tm, model, primitive);
          break;
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
          getTriangles<uint16_t>(tm, model, primitive);
          break;
        case TINYGLTF_COMPONENT_TYPE_INT:
          getTriangles<int32_t>(tm, model, primitive);
          break;
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
          getTriangles<uint32_t>(tm, model, primitive);
          break;
     }
  }
}

template <typename IndexBufferType>
void sutil::hitscan::getTriangles(sutil::hitscan::TriangleMesh& tm, const tinygltf::Model& model, const tinygltf::Primitive& primitive)
{
  // Switch based on how the position data is stored
  const tinygltf::Accessor& positionAccessor = model.accessors[primitive.attributes.at("POSITION")];
  switch(positionAccessor.componentType)
  {
    default:
    case TINYGLTF_COMPONENT_TYPE_FLOAT:
      getTrianglesInFloatForm<IndexBufferType, float>(tm, model, primitive);
      break;
    case TINYGLTF_COMPONENT_TYPE_DOUBLE:
      getTrianglesInFloatForm<IndexBufferType, double>(tm, model, primitive);
      break;
  }
}

template <typename IndexBufferType, typename PositionBufferType>
void sutil::hitscan::getTrianglesInFloatForm(sutil::hitscan::TriangleMesh& tm, const tinygltf::Model& model, const tinygltf::Primitive& primitive)
{
  // Actually get the triangles into the TriangleMesh (surprisingly simple)
  const tinygltf::Accessor& indexAccessor = model.accessors[primitive.indices];
  const tinygltf::BufferView& indexBufferView = model.bufferViews[indexAccessor.bufferView];
  const tinygltf::Buffer& indexBuffer = model.buffers[indexBufferView.buffer];

  const tinygltf::Accessor& positionAccessor = model.accessors[primitive.attributes.at("POSITION")];
  const tinygltf::BufferView& positionBufferView = model.bufferViews[positionAccessor.bufferView];
  const tinygltf::Buffer& positionBuffer = model.buffers[positionBufferView.buffer];

  const IndexBufferType* indices = reinterpret_cast<const IndexBufferType*>(&indexBuffer.data[indexBufferView.byteOffset + indexAccessor.byteOffset]);
  const PositionBufferType* positions = reinterpret_cast<const PositionBufferType*>(&positionBuffer.data[positionBufferView.byteOffset + positionAccessor.byteOffset]);

  for(int i = 0; i<indexAccessor.count; i+=3)
  {
    tm.triangles.push_back({make_float3(positions[indices[ i ]*3],positions[indices[ i ]*3+1],positions[indices[ i ]*3+2]),
                            make_float3(positions[indices[i+1]*3],positions[indices[i+1]*3+1],positions[indices[i+1]*3+2]),
                            make_float3(positions[indices[i+2]*3],positions[indices[i+2]*3+1],positions[indices[i+2]*3+2])
                           });
  }
}

void sutil::hitscan::TriangleMesh::print()
{
  for(auto tri : triangles)
  {
    std::cout << tri.p0.x << "," << tri.p0.y << "," << tri.p0.z << "|"
              << tri.p1.x << "," << tri.p1.y << "," << tri.p1.z << "|"
              << tri.p2.x << "," << tri.p2.y << "," << tri.p2.z << ":";
  }
  std::cout << "\n";
}
