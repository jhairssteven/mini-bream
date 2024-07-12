from xy_to_gps import WayptConverter

""" 1. Takes in GPS list of waypoints and the GPS origin for those waypoints
    2. Convert all to UTM and for each GPS substract the origin GPS
    3. With xy list of waypoints, input that to xy_to_gps.py"""
if __name__ == "__main__":

    base_dir = './gps_based/'
    filename = base_dir + 'Meredith_scale_1.csv'
    init_lat, init_lon, init_location = [40.4476581,-86.8682936, "Meredith_scale_1"]
    wptConv = WayptConverter()
    gps_wpts = wptConv.readXYFromFile(filename)
    utm_init_x, utm_init_y, utm_zone, utm_zone_letter = wptConv.gps_to_utm(init_lat, init_lon)
    
    xy_coordinates = []
    for gps_wpt in gps_wpts:
        utm_x, utm_y, _, _ = wptConv.gps_to_utm(gps_wpt[0], gps_wpt[1])
        xy_coordinates.append([utm_x - utm_init_x, utm_y - utm_init_y])

    wptConv.saveToFile(filename.split(".csv")[0]+f"__gps_to_xy.csv", xy_coordinates)

