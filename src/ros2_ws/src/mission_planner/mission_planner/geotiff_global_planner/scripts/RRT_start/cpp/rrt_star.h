#ifndef RRT_STAR_H
#define RRT_STAR_H

#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <iostream>

struct Pose {
    double x, y, theta;
};

struct Node {
    Pose pose;
    double cost;
    int parent; // index in nodes vector, -1 if root
    int idx;    // index in nodes vector
};

class RRTStar {
public:
    RRTStar(const unsigned char* map_data, int width, int height);
    ~RRTStar();

    std::vector<std::pair<int, int>> plan(double sx, double sy, double gx, double gy);
    void get_tree(int* out_len, double** out_x, double** out_y, double** out_theta, int** out_parent) const;
    void get_footprint_dims(double* length, double* width) const;

private:
    // Map data
    const unsigned char* map_data_;
    int width_;
    int height_;

    // RRT* Parameters
    const int MAX_ITERS = 20000;
    const double STEP = 12.0;
    const double GOAL_SAMPLE_RATE = 0.07;
    const double REWIRE_RADIUS = 40.0;
    const double GOAL_TOL = 10.0;
    const int COLLISION_SAMPLES = 6;
    const double MIN_DIST_BETWEEN_NODES = 1.0;

    // Footprint
    const double FOOTPRINT_LENGTH = 200.0;
    const double FOOTPRINT_WIDTH = 250.0;
    std::vector<std::pair<double, double>> footprint_offsets_;

    // Random number generation
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_01_;
    std::uniform_int_distribution<int> dist_w_;
    std::uniform_int_distribution<int> dist_h_;

    // Helper functions
    bool is_free(int x, int y) const;
    bool check_collision_vectorized(double x1, double y1, double x2, double y2, double theta);
    int nearest_node_idx(const std::vector<Node>& nodes, double x, double y);
    std::vector<int> nearby_nodes(const std::vector<Node>& nodes, int idx, double radius);
    Pose steer(const Pose& from, double to_x, double to_y, double step);
    bool near_existing_node(const std::vector<Node>& nodes, const Pose& p, double min_dist);

    mutable std::vector<Node> last_nodes_; // Store nodes from last plan for visualization
};

#endif // RRT_STAR_H
