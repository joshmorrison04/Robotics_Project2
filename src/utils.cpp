#include <vector>
#include <limits>

/**
 * Finds the index of the closest valid Lidar ray.
 * - Lidar gives lots of distance readings (ranges).
 * - Some are invalid (0 means no reading).
 * - We loop through all ranges and pick the smallest > 0.
 */
int findMinDist(const std::vector<float>& ranges) {
    int min_idx = -1;  // if nothing valid, stays -1
    float min_val = std::numeric_limits<float>::max(); // start huge

    for (int i = 0; i < ranges.size(); i++) {
        float r = ranges[i];
        if (r > 0 && r < min_val) {   // valid and closer than what we had
            min_val = r;
            min_idx = i;
        }
    }

    return min_idx;  // position of nearest wall ray
}

/**
 * Cross product of two 3D vectors.
 * - Here used to rotate the “point at wall” vector into
 *   a vector that runs parallel to the wall.
 */
std::vector<float> crossProduct(const std::vector<float>& a,
                                const std::vector<float>& b) {
    std::vector<float> result(3, 0.0);

    result[0] = a[1]*b[2] - a[2]*b[1];  // x
    result[1] = a[2]*b[0] - a[0]*b[2];  // y
    result[2] = a[0]*b[1] - a[1]*b[0];  // z

    return result;
}
