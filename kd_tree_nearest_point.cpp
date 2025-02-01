
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include "nanoflann/include/nanoflann.hpp"


struct Point {
    double x, y;
};

struct Odometry {
    Point pose;
    double yaw;
};

struct Waypoint {
    Point pose;
    double tangent;
};

Point subtract(const Point &a, const Point &b) {
    return {a.x - b.x, a.y - b.y};
}

// Computes the Euclidean distance between two points
double distance(const Point &a, const Point &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Normalizes a vector (point)
Point normalize(const Point &p) {
    double len = std::sqrt(p.x * p.x + p.y * p.y);
    if (len == 0) return {0, 0};  // Prevent division by zero
    return {p.x / len, p.y / len};
}

// KD-tree adaptor for nanoflann
struct PointCloud {
    std::vector<Point> pts;

    inline size_t kdtree_get_point_count() const { return pts.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return (dim == 0) ? pts[idx].x : pts[idx].y;
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

Odometry get_odometry()
{
    std::ifstream file("odometry.csv");
    std::string line;
    Odometry odometry;

    while(std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string x_str, y_str, yaw_str;
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        std::getline(ss, yaw_str, ',');
        if(!x_str.empty() && !y_str.empty() && !yaw_str.empty() && x_str != "x" /*avoids the first line*/)
        {
            odometry.pose.x = std::stod(x_str);
            odometry.pose.y = std::stod(y_str);
            odometry.yaw = std::stod(yaw_str);
        }
    }

    file.close();
    return odometry;
}

PointCloud get_trajectory()
{
    std::ifstream file("trajectory_downsampled.csv");
    std::string line;
    PointCloud cloud;

    while(std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string x_str, y_str, tangent_str;
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        if(!x_str.empty() && !y_str.empty() && x_str != "x")
        {
            cloud.pts.push_back({std::stod(x_str), std::stod(y_str)});
        }
    } 
    file.close();
    return cloud;
}

size_t get_closest_point(PointCloud cloud, Point odometry_pose)
{
    double query_point[2];

    //set query point = odometry_point
    query_point[0] = odometry_pose.x;
    query_point[1] = odometry_pose.y;

    // Build KD-Tree
    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud>, PointCloud, 2> KDTree;
    KDTree tree(2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();

    // Query point
    size_t nearest_index;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&nearest_index, &out_dist_sqr);
    tree.findNeighbors(resultSet, query_point, nanoflann::SearchParameters(10));    

    return nearest_index;
}


std::vector<double> get_tangent_angles(std::vector<Point> points)
{

    std::vector<double> tangent_angles(points.size());
    
    if (points.size() >= 2) {
        // First point: forward difference.
        Point diff = subtract(points[1], points[0]);
        Point tanVec = normalize(diff);
        tangent_angles[0] = std::atan2(tanVec.y, tanVec.x);

        // Last point: backward difference.
        diff = subtract(points.back(), points[points.size() - 2]);
        tanVec = normalize(diff);
        tangent_angles.back() = std::atan2(tanVec.y, tanVec.x);
    }

     for (size_t i = 1; i < points.size() - 1; ++i) {
        double d1 = distance(points[i], points[i - 1]);
        double d2 = distance(points[i + 1], points[i]);
        double ds = d1 + d2;  // Total distance over the two segments

        if (ds == 0) {
            tangent_angles[i] = 0;  // Fallback if points coincide
        } else {
            // Compute the central difference divided by the total arc length.
            Point diff = {
                (points[i + 1].x - points[i - 1].x) / ds,
                (points[i + 1].y - points[i - 1].y) / ds
            };
            Point tanVec = normalize(diff);
            tangent_angles[i] = std::atan2(tanVec.y, tanVec.x);
        }
    }

    return tangent_angles;
}


double get_angular_deviation(double angle1, double angle2) {
    // Compute the raw difference, then shift by π.
    double diff = angle2 - angle1 + M_PI;
    
    // Use fmod to wrap the value into the range [0, 2π)
    diff = std::fmod(diff, 2 * M_PI);
    
    // fmod can return a negative result; adjust if necessary.
    if (diff < 0)
        diff += 2 * M_PI;
    
    // Shift back by π to get a value in [-π, π]
    diff -= M_PI;
    
    // Return the absolute value to get the magnitude in [0, π]
    return std::abs(diff);
}

int main() 
{
    // Get data as external inputs
    Odometry odometry = get_odometry();
    Point odometry_pose = odometry.pose;
    PointCloud cloud = get_trajectory();

    // Find closest point to trajectory using KD-Tree from NanoFLANN
    size_t closest_point_index = get_closest_point(cloud, odometry_pose);
    Point closest_point = cloud.pts[closest_point_index];

    // Calculate lateral deviation as distance between two points
    double lateral_deviation = distance(odometry_pose, closest_point);

    // I have found the closest point on the trajectory to the odometry pose but I don't trust the result so I check if the previous or next point are closer to the odometry
    while(1)
    {
        if(closest_point_index > 0 && closest_point_index < cloud.pts.size() - 1)
        {
            double previuous_point_lateral_deviation = distance(odometry_pose, cloud.pts[closest_point_index - 1]);
            double next_point_lateral_deviation = distance(odometry_pose, cloud.pts[closest_point_index + 1]);
            if(previuous_point_lateral_deviation < lateral_deviation)
            {
                closest_point_index = closest_point_index - 1;
                closest_point = cloud.pts[closest_point_index];
                lateral_deviation = previuous_point_lateral_deviation;
            }
            else if(next_point_lateral_deviation < lateral_deviation)
            {
                closest_point_index = closest_point_index + 1;
                closest_point = cloud.pts[closest_point_index];
                lateral_deviation = next_point_lateral_deviation;
            }
            else if(next_point_lateral_deviation > lateral_deviation && previuous_point_lateral_deviation > lateral_deviation)
            {
                break;
            }
        }
        else
        {
            break;
        }
    }


    std::cout << "Lateral deviation: " << lateral_deviation << std::endl;
    
    // At this point I have the closest point on the trajectory and the lateral deviation from the odometry to the trajectory
    // Now I need to find the angular deviation between the odometry and the trajectory

    // Compute for every point on the trajectory the tangent angle
    std::vector<double> points_tangents = get_tangent_angles(cloud.pts); 
    double closest_point_tangent = points_tangents[closest_point_index];

    // Save the closest point on the trajectory and its tangent to a csv file for visualization
    std::ofstream output("closest_point.csv");
    output << "x,y,tangent\n" << closest_point.x << "," << closest_point.y << "," << closest_point_tangent << std::endl;
    output.close();

    std::cout << "odometry_pose: (" << odometry_pose.x << ", " << odometry_pose.y << ")" << std::endl;
    std::cout << "odometry yaw: " << odometry.yaw << std::endl;
    std::cout << "Closest point on trajectory: (" << closest_point.x << ", " << closest_point.y << ", " << closest_point_tangent << ")" << std::endl;

    // Finally calculate the angular deviation between the odometry and the closest point on the trajectory
    double angular_deviation = get_angular_deviation(closest_point_tangent, odometry.yaw);
    std::cout << "Angular deviation: " << angular_deviation << std::endl;

    return 0;
}
