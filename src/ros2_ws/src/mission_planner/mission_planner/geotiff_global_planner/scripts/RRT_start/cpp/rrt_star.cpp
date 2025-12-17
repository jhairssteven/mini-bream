#include "rrt_star.h"

RRTStar::RRTStar(const unsigned char* map_data, int width, int height)
    : map_data_(map_data), width_(width), height_(height), rng_(std::random_device{}()), dist_01_(0.0, 1.0), dist_w_(0, width - 1), dist_h_(0, height - 1) {
    
    // Precompute footprint offsets
    double L = FOOTPRINT_LENGTH;
    double W = FOOTPRINT_WIDTH;
    double halfL = L / 2.0;
    double halfW = W / 2.0;

    // Vertices
    footprint_offsets_.push_back({halfL, halfW});
    footprint_offsets_.push_back({halfL, -halfW});
    footprint_offsets_.push_back({-halfL, -halfW});
    footprint_offsets_.push_back({-halfL, halfW});

    // Midpoints
    footprint_offsets_.push_back({halfL, 0});
    footprint_offsets_.push_back({0, -halfW});
    footprint_offsets_.push_back({-halfL, 0});
    footprint_offsets_.push_back({0, halfW});

    // Center
    footprint_offsets_.push_back({0, 0});
}

RRTStar::~RRTStar() {}

void RRTStar::get_footprint_dims(double* length, double* width) const {
    *length = FOOTPRINT_LENGTH;
    *width = FOOTPRINT_WIDTH;
}

bool RRTStar::is_free(int x, int y) const {
    if (x < 0 || y < 0 || x >= width_ || y >= height_) return false;
    // Assuming map_data is row-major, 0 is obstacle, >250 is free (based on python code)
    // Python: free = (img_arr > 250) -> converted to 0/1 in python wrapper
    return map_data_[y * width_ + x] > 0;
}

bool RRTStar::check_collision_vectorized(double x1, double y1, double x2, double y2, double theta) {
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);

    // Precompute rotated offsets
    std::vector<std::pair<double, double>> rotated_offsets;
    rotated_offsets.reserve(footprint_offsets_.size());
    for (const auto& off : footprint_offsets_) {
        rotated_offsets.push_back({
            off.first * cos_t - off.second * sin_t,
            off.first * sin_t + off.second * cos_t
        });
    }

    // Sample points along the line
    for (int i = 1; i <= COLLISION_SAMPLES; ++i) {
        double t = (double)i / COLLISION_SAMPLES;
        double px = x1 + (x2 - x1) * t;
        double py = y1 + (y2 - y1) * t;

        // Check all footprint points
        for (const auto& off : rotated_offsets) {
            int ix = std::round(px + off.first);
            int iy = std::round(py + off.second);
            
            // Bounds check
            if (ix < 0 || iy < 0 || ix >= width_ || iy >= height_) {
                 // Per python logic: footprint polygon can be outside map size EXCEPT center
                 // But let's stick to the vectorized logic which was stricter/safer or similar
                 // Python vectorized: if center out of bounds -> collision. if others out -> free.
                 // Let's implement that exactly.
                 
                 // If it's the center point (last one)
                 if (&off == &rotated_offsets.back()) return false; 
                 continue; // others out of bounds is fine
            }
            
            if (map_data_[iy * width_ + ix] == 0) return false;
        }
    }
    return true;
}

int RRTStar::nearest_node_idx(const std::vector<Node>& nodes, double x, double y) {
    int best_idx = -1;
    double best_dist_sq = std::numeric_limits<double>::max();

    for (int i = 0; i < nodes.size(); ++i) {
        double dx = nodes[i].pose.x - x;
        double dy = nodes[i].pose.y - y;
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_idx = i;
        }
    }
    return best_idx;
}

std::vector<int> RRTStar::nearby_nodes(const std::vector<Node>& nodes, int idx, double radius) {
    std::vector<int> nearby;
    double r_sq = radius * radius;
    double x = nodes[idx].pose.x;
    double y = nodes[idx].pose.y;

    for (int i = 0; i < nodes.size(); ++i) {
        double dx = nodes[i].pose.x - x;
        double dy = nodes[i].pose.y - y;
        if (dx * dx + dy * dy <= r_sq) {
            nearby.push_back(i);
        }
    }
    return nearby;
}

Pose RRTStar::steer(const Pose& from, double to_x, double to_y, double step) {
    double dx = to_x - from.x;
    double dy = to_y - from.y;
    double dist = std::hypot(dx, dy);

    if (dist < 1e-6) return from;

    double scale = std::min(step / dist, 1.0);
    return {from.x + dx * scale, from.y + dy * scale, std::atan2(dy, dx)};
}

bool RRTStar::near_existing_node(const std::vector<Node>& nodes, const Pose& p, double min_dist) {
    double min_dist_sq = min_dist * min_dist;
    for (const auto& n : nodes) {
        double dx = n.pose.x - p.x;
        double dy = n.pose.y - p.y;
        if (dx * dx + dy * dy < min_dist_sq) return true;
    }
    return false;
}

std::vector<std::pair<int, int>> RRTStar::plan(double sx, double sy, double gx, double gy) {
    if (!is_free((int)sx, (int)sy) || !is_free((int)gx, (int)gy)) {
        std::cerr << "Start or Goal on obstacle" << std::endl;
        return {};
    }

    double start_theta = std::atan2(gy - sy, gx - sx);
    std::vector<Node> nodes;
    nodes.reserve(MAX_ITERS + 100);
    nodes.push_back({{sx, sy, start_theta}, 0.0, -1, 0});

    int closest_idx = 0;
    double closest_dist = std::hypot(sx - gx, sy - gy);
    int goal_node_idx = -1;

    for (int i = 0; i < MAX_ITERS; ++i) {
        double rx, ry;
        if (dist_01_(rng_) < GOAL_SAMPLE_RATE) {
            rx = gx;
            ry = gy;
        } else {
            rx = dist_w_(rng_);
            ry = dist_h_(rng_);
        }

        if (!is_free((int)rx, (int)ry)) continue;

        int nearest_idx = nearest_node_idx(nodes, rx, ry);
        Pose new_pose = steer(nodes[nearest_idx].pose, rx, ry, STEP);

        if (near_existing_node(nodes, new_pose, MIN_DIST_BETWEEN_NODES)) continue;

        if (!check_collision_vectorized(nodes[nearest_idx].pose.x, nodes[nearest_idx].pose.y, 
                                      new_pose.x, new_pose.y, new_pose.theta)) {
            continue;
        }

        double new_cost = nodes[nearest_idx].cost + std::hypot(new_pose.x - nodes[nearest_idx].pose.x, 
                                                             new_pose.y - nodes[nearest_idx].pose.y);
        
        int new_idx = nodes.size();
        nodes.push_back({new_pose, new_cost, nearest_idx, new_idx});

        // Rewire
        std::vector<int> neighbors = nearby_nodes(nodes, new_idx, REWIRE_RADIUS);
        
        // Choose best parent
        for (int ni : neighbors) {
            if (ni == new_idx) continue;
            double d = std::hypot(new_pose.x - nodes[ni].pose.x, new_pose.y - nodes[ni].pose.y);
            double candidate_cost = nodes[ni].cost + d;
            
            if (candidate_cost + 1e-8 < nodes[new_idx].cost) {
                if (check_collision_vectorized(nodes[ni].pose.x, nodes[ni].pose.y, 
                                             new_pose.x, new_pose.y, new_pose.theta)) {
                    nodes[new_idx].cost = candidate_cost;
                    nodes[new_idx].parent = ni;
                }
            }
        }

        // Rewire neighbors
        for (int ni : neighbors) {
            if (ni == new_idx) continue;
            double d = std::hypot(nodes[ni].pose.x - new_pose.x, nodes[ni].pose.y - new_pose.y);
            double candidate_cost = nodes[new_idx].cost + d;
            
            if (candidate_cost + 1e-8 < nodes[ni].cost) {
                // Note: using nodes[ni].pose.theta for collision check as we are going TO ni
                // But actually the edge is bidirectional for collision usually, but strictly it's directed.
                // The python code checks edge from new_pose to nnode.
                // Python: _edge_collision_free(new_pose, nnode.pose)
                // Theta is calculated from direction.
                double theta = std::atan2(nodes[ni].pose.y - new_pose.y, nodes[ni].pose.x - new_pose.x);
                if (check_collision_vectorized(new_pose.x, new_pose.y, nodes[ni].pose.x, nodes[ni].pose.y, theta)) {
                    nodes[ni].parent = new_idx;
                    nodes[ni].cost = candidate_cost;
                }
            }
        }

        // Goal check
        double dist_to_goal = std::hypot(new_pose.x - gx, new_pose.y - gy);
        if (dist_to_goal < closest_dist) {
            closest_dist = dist_to_goal;
            closest_idx = new_idx;
        }

        if (dist_to_goal <= GOAL_TOL) {
            double goal_theta = std::atan2(gy - new_pose.y, gx - new_pose.x);
            if (check_collision_vectorized(new_pose.x, new_pose.y, gx, gy, goal_theta)) {
                nodes.push_back({{gx, gy, goal_theta}, nodes[new_idx].cost + dist_to_goal, new_idx, (int)nodes.size()});
                goal_node_idx = nodes.size() - 1;
                std::cout << "RRT* reached goal in " << i << " iterations" << std::endl;
                break;
            }
        }
    }

    // Reconstruct path
    std::vector<std::pair<int, int>> path;
    int cur = (goal_node_idx != -1) ? goal_node_idx : closest_idx;
    
    while (cur != -1) {
        path.push_back({(int)std::round(nodes[cur].pose.x), (int)std::round(nodes[cur].pose.y)});
        cur = nodes[cur].parent;
    }
    std::reverse(path.begin(), path.end());
    
    // Store nodes for visualization retrieval
    last_nodes_ = nodes;

    return path;
}

void RRTStar::get_tree(int* out_len, double** out_x, double** out_y, double** out_theta, int** out_parent) const {
    *out_len = last_nodes_.size();
    if (last_nodes_.empty()) {
        *out_x = nullptr;
        *out_y = nullptr;
        *out_theta = nullptr;
        *out_parent = nullptr;
        return;
    }

    *out_x = new double[*out_len];
    *out_y = new double[*out_len];
    *out_theta = new double[*out_len];
    *out_parent = new int[*out_len];

    for (size_t i = 0; i < last_nodes_.size(); ++i) {
        (*out_x)[i] = last_nodes_[i].pose.x;
        (*out_y)[i] = last_nodes_[i].pose.y;
        (*out_theta)[i] = last_nodes_[i].pose.theta;
        (*out_parent)[i] = last_nodes_[i].parent;
    }
}
