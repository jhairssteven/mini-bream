import csv
import utm

class WayptConverter:
    def gps_to_utm(self, lat, long):
        """This function will take a gps value and convert it to a utm coordinate and return the resulting
        norths and easts
        Input:
            lat = Latitude of vehicle in degrees North (negative is south)
            long = Latitude of vehcile in degrees East (negative is west)
        Output:
            Norths = the position of the vehicle in meters north of the origin of the current UTM zone
            Easts = the position of the vehicle in meters east of the origin o fthe current UTM Zone
            Zone = The current operational UTM zone
        """
        easts, norths, zone_num, zone_letter = utm.from_latlon(lat, long)
        #zone_letter = "T"
        return easts, norths, zone_num, zone_letter
    
    def readXYFromFile(self, filename):
        """ Get the x,y coordinates into a 2d-array  from 'filename' """
        xs = []  # initialize temporary array of latitude points to read in - [deg]
        ys = []  # initialize temporary array of longitude points to read in - [deg]
        xy_wypts = []
        movtypes = []  # initialize a list of movement points for each waypoint - [""]
        times = []  # initialize a list of times from start for each waypoint - [""]
        # Import waypoints that have been manual defined in the csv file
        with open(filename, 'r') as csvfile:  # Do all following with the waypoints CSV file and name csvfile
            # dialect = csv.Sniffer().sniff(csvfile.read(1024))
            # csvfile.seek(0)
            reader = csv.reader(csvfile)  # Use the CSV reader to read the CSV file
            next(reader, None)
            row_counter = 1  # count number of rows
            for row in reader:  # for each row in the CSV file
                if row_counter > 2:  # if we are reading waypoints
                    xy_wypts.append([float(row[0]), float(row[1])])
                    #xs.append(float(row[0]))  # Append the new goal latitude to the class's list
                    #ys.append(float(row[1]))  # Append the new goal latitude to the class's list
                    movtypes.append(str(row[2]))  # Append the new thing to do at the point
                    times.append(float(row[3]))  # append the new times of each waypoint
                row_counter += 1  # next row
        return xy_wypts

    def xy_to_utm(self, xy_coords, utm_init_x, utm_init_y, utm_zone, utm_zone_letter):
        """ Return xy_coords [[x,y]] in UTM format [[utm_x, utm_y, utm_zone], ], with respect to initial utm_init_x and utm_init_y """
        utm_waypts = []
        for xy_wp in xy_coords:
            utm_waypts.append([xy_wp[0] + utm_init_x, xy_wp[1] + utm_init_y, utm_zone, utm_zone_letter])
        return utm_waypts
    
    def utm_to_gps(self, utm_waypts):
        """ Return utm_waypts [[utm_x, utm_y, utm_zone], ] in GPS [[lat, lon],] format """
        gps_wypts = []
        for utm_wp in utm_waypts:
            [utm_x, utm_y, utm_zone, utm_zone_letter] = utm_wp
            self.gps_lat, self.gps_lon = utm.to_latlon(utm_x, utm_y, utm_zone, utm_zone_letter)
            gps_wypts.append([self.gps_lat, self.gps_lon])
        return gps_wypts

    def saveToFile(self, filename, gps_waypts):
        with open(filename, 'w') as f:
            headers = []
            headers.append(['docking station','loop','timed','loop time','dubins path','Catamaran'])
            headers.append(['FALSE','TRUE','TRUE','10800','FALSE','FALSE'])
            headers.append(['lat','long','action','time to point','hold-radius','docking traj'])
            for gps_waypt in gps_waypts:
                gps_waypt.append('move')
                gps_waypt.append('1000')
            
            import pandas as pd
            df = pd.DataFrame(headers[1:], columns=headers[0])
            df.to_csv(filename, index=False)
            df2 = pd.DataFrame(gps_waypts[0:], columns=gps_waypts[0])  # Create DataFrame from your data
            df2.to_csv(filename, mode='a', index=False, header=False)

""" This script reads the x,y coordinates from a csv file and an initial GPS (lat, lon) pair
and generates the equivalent GPS coordinates for that (x,y) path to fed into the backseat.
This way you can create trajectory profiles agnostic to the initial GPS position of the vehicle."""
if __name__ == "__main__":

    base_dir = './xy_based/'
    filename = base_dir + 'Meredith_scale_1__gps_to_xy.csv'
    
    init_lat, init_lon, init_location = [-33.722765834294044, 150.67398711024646, 'vrx_gazebo'] # vrx_gazebo
    #init_lat, init_lon, init_location = [40.392243, -86.760399, 'kepner'] # kepner
    #init_lat, init_lon, init_location = [40.402565002441406, -86.84538269042969, 'kepner2'] # kepner2
    #init_lat, init_lon, init_location = [40.44755935668945, -86.86846160888672, "lakefield"]
    #init_lat, init_lon, init_location = [40.44771957397461, -86.86841583251953, "lakefield_may23"]
    #init_lat, init_lon, init_location = [40.4026077, -86.8454623, "kepnerEntrance_jun8"] # Kepner main entrance
    #init_lat, init_lon, init_location = [40.447628699999996,-86.8682698, "lakefield_jun18"]
    wptConv = WayptConverter()
    xy_wpts = wptConv.readXYFromFile(filename)
    utm_init_x, utm_init_y, utm_zone, utm_zone_letter = wptConv.gps_to_utm(init_lat, init_lon)
    utm_wypts = wptConv.xy_to_utm(xy_wpts, utm_init_x, utm_init_y, utm_zone, utm_zone_letter)
    gps_wypts = wptConv.utm_to_gps(utm_wypts)
    wptConv.saveToFile(filename.split(".csv")[0]+f"__gps_init_at_{init_location}.csv", gps_wypts)

