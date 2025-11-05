# geo_transform_utils.py
from pyproj import CRS, Transformer
import math
import numpy as np
import rasterio


class GeoImageTransformer:
    """
    Handles pixel <-> GPS coordinate transformations for a local map image.

    This class allows conversion from image pixel coordinates to latitude/longitude
    (and vice versa) given a reference pixel with known GPS location, the physical
    size of the image area in meters, and optional image rotation.

    The class supports snapping coordinates to the nearest valid pixel and accounts
    for typical image conventions where the y-axis increases downward.

    Can be initialized from:
        1. Reference pixel + known GPS + image size in pixels + physical size in meters
        2. A georeferenced raster (GeoTIFF) with optional reference pixel
    """

    def __init__(self, ref_pixel=None, ref_latlon=None,
                 image_size_px=None, scale_xy=None, 
                 rotation_deg=0.0, y_axis_down=True,
                 geotiff_path=None, geotiff_ref_pixel=(0, 0)):
        """
        Initialize the transformer with image and geospatial reference information.

        Parameters
        ----------
        ref_pixel : tuple of int (u0, v0)
            Pixel coordinates in the image whose latitude/longitude is known.
            For example, (0, 0) for the top-left pixel or any arbitrary pixel.
        
        ref_latlon : tuple of float (lat0, lon0)
            Latitude and longitude (degrees) of the reference pixel `ref_pixel`.
        
        image_size_px : tuple of int (width_px, height_px)
            Size of the image in pixels, e.g., (1280, 720).
        
        scale_xy : tuple of float (x scale, y scale ) pixels / m
            This defines the meters-per-pixel scaling in both directions.
        
        rotation_deg : float, optional, default=0.0
            Clockwise rotation of the image in degrees relative to North-up.
            0.0 means image +x points East and +y points South (typical image axes).
        
        y_axis_down : bool, optional, default=True
            True if the image's vertical pixel index increases downward (common for images),
            False if vertical index increases upward.
        geotiff_path : str, optional
            Path to a GeoTIFF file. If provided, reference info is extracted
            automatically from the raster.

        geotiff_ref_pixel : tuple (u, v), default=(0,0)
            Pixel in the GeoTIFF to use as the reference. Defaults to top-left.

        Notes
        -----
        - If geotiff_path is provided, ref_pixel, ref_latlon, image_size_px,
          and scale_xy are inferred from the GeoTIFF. Otherwise, they
          must be provided manually.
        - The transformer uses an Azimuthal Equidistant (AEQD) projection centered
          at `ref_latlon` to perform accurate metric offsets for pixel <-> GPS conversion.
        - Once initialized, the instance can efficiently convert single or multiple
          pixel coordinates to GPS coordinates and vice versa.
        """

        self.rotation_deg = rotation_deg
        self.y_axis_down = y_axis_down

        if geotiff_path:
            self._init_from_geotiff(geotiff_path, geotiff_ref_pixel)
        else:
            if None in (ref_pixel, ref_latlon, image_size_px, scale_xy):
                raise ValueError("Must provide ref_pixel, ref_latlon, image_size_px, and scale_xy if no GeoTIFF is given.")
            self.ref_pixel = ref_pixel
            self.ref_latlon = ref_latlon
            self.image_size_px = image_size_px
            self.physical_size_m = (scale_xy[0] * image_size_px[0], scale_xy[1] * image_size_px[1])

        # Precompute affine and projection data
        self._setup_transformers()

    def _init_from_geotiff(self, geotiff_path, ref_pixel):
        """Load georeference info from a GeoTIFF using rasterio."""
        with rasterio.open(geotiff_path) as ds:
            self.image_size_px = (ds.width, ds.height)

            # Compute physical size in meters
            W_m = ds.transform.a * ds.width
            H_m = ds.transform.e * ds.height
            self.physical_size_m = (abs(W_m), abs(H_m))

            self.ref_pixel = ref_pixel

            # Transform pixel to GPS using rasterio
            col, row = ref_pixel
            lon, lat = ds.transform * (col, row)
            self.ref_latlon = (lat, lon)

    def _setup_transformers(self):
        lat0, lon0 = self.ref_latlon
        self.theta = math.radians(-self.rotation_deg)
        self.cos_t = math.cos(self.theta)
        self.sin_t = math.sin(self.theta)

        # Projection centered at reference point
        self.aeqd_crs = CRS.from_proj4(f"+proj=aeqd +lat_0={lat0} +lon_0={lon0} +datum=WGS84 +units=m")
        self.wgs84 = CRS.from_epsg(4326)
        self.to_aeqd = Transformer.from_crs(self.wgs84, self.aeqd_crs, always_xy=True)
        self.to_wgs84 = Transformer.from_crs(self.aeqd_crs, self.wgs84, always_xy=True)

        # Reference position in projected coordinates
        lon0, lat0 = self.ref_latlon[1], self.ref_latlon[0]
        self.ref_x, self.ref_y = self.to_aeqd.transform(lon0, lat0)

        # Scale factors (meters per pixel)
        W_px, H_px = self.image_size_px
        W_m, H_m = self.physical_size_m
        self.sx = W_m / float(W_px)
        self.sy = H_m / float(H_px)

    # === Public methods ===
    def pixels_to_latlon(self, pixels):
        """Convert a list of (u, v) pixels to list of (lat, lon)."""
        u0, v0 = self.ref_pixel
        out = []
        for (u, v) in pixels:
            du = float(u) - float(u0)
            dv = float(v) - float(v0)

            dx_img = du * self.sx
            dy_img = dv * self.sy
            if self.y_axis_down:
                dy_img = -dy_img

            dx_en = dx_img * self.cos_t - dy_img * self.sin_t
            dy_en = dx_img * self.sin_t + dy_img * self.cos_t

            x_m = self.ref_x + dx_en
            y_m = self.ref_y + dy_en
            lon, lat = self.to_wgs84.transform(x_m, y_m)
            out.append((lat, lon))
        return out

    def latlon_to_pixel(self, latlon, snap_to_bounds=True):
        """Convert one GPS coordinate (lat, lon) to image pixel (u, v)."""
        lat, lon = latlon
        tgt_x, tgt_y = self.to_aeqd.transform(lon, lat)

        dx_en = tgt_x - self.ref_x
        dy_en = tgt_y - self.ref_y

        dx_img = dx_en * self.cos_t + dy_en * self.sin_t
        dy_img = -dx_en * self.sin_t + dy_en * self.cos_t

        if self.y_axis_down:
            dy_img = -dy_img

        u0, v0 = self.ref_pixel
        u = u0 + dx_img / self.sx
        v = v0 + dy_img / self.sy

        # Snap to valid range
        W_px, H_px = self.image_size_px
        if snap_to_bounds:
            u_clamped = min(max(u, 0), W_px - 1)
            v_clamped = min(max(v, 0), H_px - 1)
        else:
            u_clamped, v_clamped = u, v

        px_offset = math.hypot(u - u_clamped, v - v_clamped)
        m_offset = math.hypot((u - u_clamped) * self.sx, (v - v_clamped) * self.sy)

        return {
            "u": u_clamped,
            "v": v_clamped,
            "within_bounds": (u == u_clamped and v == v_clamped),
            "pixel_offset": px_offset,
            "meter_offset": m_offset
        }
