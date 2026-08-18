"""LiDAR point-cloud filtering pipeline.

Stages, chained by ``lidar_filtering.launch.py``:

1. ``self_filter`` — drop boat hull / mast returns
2. ``ransac`` — sequential near-horizontal waterline planes
3. ``pc_clustering`` — DBSCAN size-gate to drop sparse noise

See ``README.md`` in this directory for topics and launch.
"""
