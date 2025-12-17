#!/bin/bash
g++ -O3 -shared -fPIC -o librrt_star.so cpp/rrt_star.cpp cpp/bindings.cpp
echo "Compilation complete: librrt_star.so"

# -O3: Enables high-level optimizations (critical for performance).
# -shared: Produces a shared library (.so file) instead of an executable.
# -fPIC: Generates Position Independent Code, which is required for shared libraries.
# -o librrt_star.so: Specifies the output filename.