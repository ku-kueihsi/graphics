#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <tuple>
#include <optional>

using namespace std;
typedef array<double, 3> gVec3;

#define ABS(a) (a < 0 ? -a : a)
#define MAX(a, b) (a > b ? a : b)
#define MIN(a, b) (a < b ? a : b)
#define ESP 1e-9

gVec3 operator+(const gVec3& vec1, const gVec3& vec2) {
  gVec3 res;
  for (int i = 0; i < 3; i++) {
    res[i] = vec1[i] + vec2[i];
  }
  return res;
}

gVec3 operator-(const gVec3& vec1, const gVec3& vec2) {
  gVec3 res;
  for (int i = 0; i < 3; i++) {
    res[i] = vec1[i] - vec2[i];
  }
  return res;
}

gVec3 operator*(double s, const gVec3& vec1) {
  gVec3 res;
  for (int i = 0; i < 3; i++) {
    res[i] = s * vec1[i];
  }
  return res;
}

gVec3 operator*(const gVec3& vec1, double s) {
  gVec3 res;
  for (int i = 0; i < 3; i++) {
    res[i] = s * vec1[i];
  }
  return res;
}


double dot(const gVec3& vec1, const gVec3& vec2) {
  double res = 0.0;
  for (int i = 0; i < 3; i++) {
    res += vec1[i] * vec2[i];
  }
  return res;
}

gVec3 cross(const gVec3& vec1, const gVec3& vec2) {
  //  1_0 1_1 1_2
  //  2_0 2_1 2_2
  //    0   1   2
  gVec3 res = gVec3();
  for (int i = 0; i < 3; i++) {
    res[i] = vec1[(i + 1) % 3] * vec2[(i + 2) % 3] - vec1[(i + 2) % 3] * vec2[(i + 1) % 3];
  }
  return res;
}

optional<gVec3> rayTriInter(
    const gVec3 &orig,
    const gVec3 &direct,
    const vector<gVec3> &triangle)
{
  // o + d*t = v0 + (v1 - v0) * u + (v2 - v0) * v
  // where t >= 0 u, v, u + v in [0, 1]

  // o - v0 + d*t = (v1 - v0) * u + (v2 - v0) * v
  // p + d*t = e1*u + e2*v
  // de1 = d x e1, de2 = d x e1, e1e2 = e1 x e2
  // de1.p = de1.e2*v
  // de2.p = de2.e1*u
  // -e1e2.p = e1e2.d*t
  gVec3 e1 = triangle[1] - triangle[0];
  gVec3 e2 = triangle[2] - triangle[0];
  gVec3 p = orig - triangle[0];
  gVec3 de1 = cross(direct, e1);
  double a = dot(de1, e2);
  if (ABS(a) < ESP) {
    return nullopt;
  }
  double v = dot(de1, p) / a;
  if (v < 0 or v > 1) {
    return nullopt;
  }
  gVec3 de2 = cross(direct, e2);
  a = dot(de2, e1);
  if (ABS(a) < ESP) {
    return nullopt;
  }
  double u = dot(de2, p) / a;
  if (u < 0 or u > 1 or u + v < 0 or u + v > 1) {
    return nullopt;
  }
  gVec3 e1e2 = cross(e1, e2);
  a = dot(e1e2, direct);
  if (ABS(a) < ESP) {
    return nullopt;
  }
  double t = -dot(e1e2, p) / a;
  if (t < 0) {
    return nullopt;
  }
  // gVec3 tmp = orig + (t * direct);
  // gVec3 tmp2 = triangle[0] + u * e1 + v * e2;
  return orig + (t * direct);
}

std::optional<gVec3> rayTriInter1( const gVec3 &ray_origin,
    const gVec3 &ray_vector,
    const vector<gVec3>& triangle)
{
    constexpr float epsilon = std::numeric_limits<float>::epsilon();

    gVec3 edge1 = triangle[1] - triangle[0];
    gVec3 edge2 = triangle[2] - triangle[0];
    gVec3 ray_cross_e2 = cross(ray_vector, edge2);
    float det = dot(edge1, ray_cross_e2);

    if (det > -epsilon && det < epsilon)
        return nullopt;    // This ray is parallel to this triangle.

    float inv_det = 1.0 / det;
    gVec3 s = ray_origin - triangle[0];
    float u = inv_det * dot(s, ray_cross_e2);

    if ((u < 0 && abs(u) > epsilon) || (u > 1 && abs(u-1) > epsilon))
        return nullopt;

    gVec3 s_cross_e1 = cross(s, edge1);
    float v = inv_det * dot(ray_vector, s_cross_e1);

    if ((v < 0 && abs(v) > epsilon) || (u + v > 1 && abs(u + v - 1) > epsilon))
        return nullopt;

    // At this stage we can compute t to find out where the intersection point is on the line.
    float t = inv_det * dot(edge2, s_cross_e1);

    if (t > epsilon) // ray intersection
    {
        return  ray_origin + ray_vector * t;
    }
    else // This means that there is a line intersection but not a ray intersection.
        return nullopt;
}

// int main() {
//   cout << "Hello" << endl;
//   return 0;
// }

// Helper function for approximate comparisons
bool approxEqual(const gVec3& a, const gVec3& b, double epsilon = 1e-6) {
    return fabs(a[0] - b[0]) < epsilon &&
           fabs(a[1] - b[1]) < epsilon &&
           fabs(a[2] - b[2]) < epsilon;
}

void runTest(const string& name, 
             const gVec3& orig, const gVec3& direct,
             const vector<gVec3>& triangle,
             bool expectedResult, const gVec3& expectedInter) {
    gVec3 inter;
    optional<gVec3> result = rayTriInter(orig, direct, triangle);
    
    bool passed = (result.has_value() == expectedResult);
    if (passed && expectedResult) {
        passed = approxEqual(result.value(), expectedInter);
    }

    cout << (passed ? "[PASS] " : "[FAIL] ") << name << endl;
    if (!passed) {
        cout << "  Expected: " << expectedResult << " ";
        if (expectedResult) {
            cout << "(" << expectedInter[0] << ", " 
                      << expectedInter[1] << ", "
                      << expectedInter[2] << ")";
        }
        cout << "\n  Actual:   " << result.has_value() << " ";
        if (result) {
            cout << "(" << inter[0] << ", " 
                      << inter[1] << ", "
                      << inter[2] << ")";
        }
        cout << endl;
    }
}

int main() {
    vector<tuple<string, gVec3, gVec3, vector<gVec3>, bool, gVec3>> tests = {
        // Test 1: Intersection at Vertex
        {"1. Vertex Intersection",
         {0.2, 0.2, 2}, {0, 0, -1},
         {{0,0,0}, {1,0,0}, {0,1,0}},
         true, {0,0,0}},

        // // Test 2: Edge Intersection
        // {"2. Edge Intersection",
        //  {0.5, 0.5, 2}, {0, 0, -1},
        //  {{0,0,0}, {1,0,0}, {0,1,0}},
        //  true, {0.5, 0.5, 0}},

        // // Test 3: Ray Starts Inside
        // {"3. Ray Inside Triangle",
        //  {0.3, 0.3, 0}, {0, 0, 1},
        //  {{0,0,0}, {1,0,0}, {0,1,0}},
        //  true, {0.3, 0.3, 0}},

        // Test 4: Thin Sliver Triangle
        {"4. Thin Sliver Triangle",
         {0.5, 0.0001, 2}, {0, 0, -1},
         {{0,0,0}, {1,0,0}, {0,1e-6,0}},
         true, {0.5, 0.0001, 0}},

        // Test 5: Non-Unit Direction
        {"5. Non-Unit Direction",
         {0,0,3}, {0,0,-2},
         {{0,0,0}, {1,0,0}, {0,1,0}},
         true, {0,0,0}},

        // Test 6: Triangle Behind Ray
        {"6. Triangle Behind",
         {0,0,1}, {0,0,1},
         {{0,0,2}, {1,0,2}, {0,1,2}},
         false, {}},

        // Test 7: Large Coordinates
        {"7. Large Coordinates",
         {1e6, 1e6, 1e6+1}, {0,0,-1},
         {{1e6,1e6,1e6}, {1e6+1,1e6,1e6}, {1e6,1e6+1,1e6}},
         true, {1e6,1e6,1e6}},

        // Test 8: Degenerate Triangle
        {"8. Degenerate Triangle",
         {0.5,0.5,1}, {0,0,-1},
         {{0,0,0}, {1,0,0}, {2,0,0}},
         false, {}},

        // Test 9: Parallel Ray
        {"9. Parallel Ray",
         {0,0,1}, {1,0,0},
         {{0,0,0}, {1,0,0}, {0,1,0}},
         false, {}},

        // Test 10: 3D Oblique Intersection
        {"10. 3D Oblique",
         {1,1,1}, {-1,-1,-1},
         {{0,0,0}, {2,0,0}, {0,2,0}},
         true, {0,0,0}}
    };

    for (const auto& [name, orig, direct, tri, expRes, expInter] : tests) {
        runTest(name, orig, direct, tri, expRes, expInter);
    }

    return 0;
}