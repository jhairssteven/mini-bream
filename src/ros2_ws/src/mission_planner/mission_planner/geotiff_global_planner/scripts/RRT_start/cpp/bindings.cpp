#include "rrt_star.h"
#include <cstring>

extern "C" {
    RRTStar* RRTStar_new(const unsigned char* map_data, int width, int height) {
        return new RRTStar(map_data, width, height);
    }

    void RRTStar_delete(RRTStar* ptr) {
        if (ptr) delete ptr;
    }

    void RRTStar_plan(RRTStar* ptr, double sx, double sy, double gx, double gy, 
                     int* out_len, int** out_x, int** out_y) {
        if (!ptr) return;
        
        std::vector<std::pair<int, int>> path = ptr->plan(sx, sy, gx, gy);
        
        *out_len = path.size();
        if (path.empty()) {
            *out_x = nullptr;
            *out_y = nullptr;
            return;
        }

        *out_x = new int[path.size()];
        *out_y = new int[path.size()];

        for (size_t i = 0; i < path.size(); ++i) {
            (*out_x)[i] = path[i].first;
            (*out_y)[i] = path[i].second;
        }
    }

    void RRTStar_free_path(int* x, int* y) {
        if (x) delete[] x;
        if (y) delete[] y;
    }

    void RRTStar_get_footprint_dims(RRTStar* ptr, double* length, double* width) {
        if (ptr) ptr->get_footprint_dims(length, width);
    }

    void RRTStar_get_tree(RRTStar* ptr, int* out_len, double** out_x, double** out_y, double** out_theta, int** out_parent) {
        if (ptr) ptr->get_tree(out_len, out_x, out_y, out_theta, out_parent);
    }

    void RRTStar_free_tree(double* x, double* y, double* theta, int* parent) {
        if (x) delete[] x;
        if (y) delete[] y;
        if (theta) delete[] theta;
        if (parent) delete[] parent;
    }
}
