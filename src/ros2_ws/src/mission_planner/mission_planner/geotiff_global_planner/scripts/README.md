# GeoTiff global planner
**Core idea:** Given a GeoTiff binary image, plan a path using A* algorithm on a traversable area.
## Dependencies

```bash
python-motion-planning 
pyastar2d # (build from source)
rasterio
pyproj
```

## pyastar2d
Clone the [repository](https://github.com/jhairssteven/pyastar2d) and build from source 

```bash
cd pyastar2d
pip install -e .
```