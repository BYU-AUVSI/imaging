'''
            BYU AUVSI-SUAS Capstone
         Target Geolocation Algorithm
              Connor Olsen, 2019
'''

import numpy as np
import math as math
from math import tan
from math import atan
from math import atan2
from math import cos
from math import acos
from math import radians
from math import degrees
from math import sin
from math import asin
from math import sqrt
from geographiclib.geodesic import Geodesic

class targetGeolocation:
    """docstring for targetGeolocation."""
    def __init__(self, gnd_lat, gnd_lon):
        self.lat_gnd = gnd_lat
        self.lon_gnd = gnd_lon
        self.radiusOfEarth = 6378137.0
        self.f = 1/298.257223563            #Flattening of the ellipsoid (WGS-84)
        self.b = (1-self.f) * self.radiusOfEarth                  
        self.maxIterations = 250
        self.maxChange = 10**(-12)
        self.geod = Geodesic.WGS84  #TODO: Get rid of these selfs and be consistent with Geodesic

    """ OLD METHOD
    Calculates the meters between two GPS coordinates using the Haversine Formula
    @type lat1: float
    @param lat1: The latitude of the first GPS coordinate

    @type lon1: float
    @param lon1: The longitude of the first GPS coordinate

    @type lat2: float
    @param lat2: The latitude of the second GPS coordinate

    @type lon2: float
    @param lon2: The longitude of the first GPS coordinate

    @rtype: array of floats(4)
    @return: Distance north (meters), distande east (meters), total distance (meters), angle (0 deg = East)
    """
    def _GPStoMeters(self, lat1, lon1, lat2, lon2):
        # d2r = 0.0174532925199433       # Scale factor for degrees 
        dlong = math.radians(lon2 - lon1)
        dlat = math.radians(lat2 - lat1) 
        a = (math.sin(dlat/2.0))**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * (math.sin(dlong/2.0))**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        radius = 6378.137 * c #Distance between points in kilometers
        d_meters = radius * 1000
        dy = lat2 - lat1
        dx = math.cos(math.radians(lat1)) * (lon2 - lon1)
        angle = math.atan2(dy, dx)
        east_dis_meters = d_meters * math.cos(angle)
        north_dis_meters = d_meters * math.sin(angle)
        returnvalues = np.array([north_dis_meters, east_dis_meters, d_meters, angle])
        return returnvalues

    """ OLD METHOD
    Calculates GPS coordinates given a starting coordinate and meters north and east
    @type Lat: float
    @param Lat: The latitude of the GPS coordinate

    @type Lon: float
    @param Lon: The longitude of the GPS coordinate

    @type north_displacement: float
    @param north_displacement: The distance north of the given coordinates (meters)

    @type east_displacement: float
    @param east_displacement: The distance east of the given coordinates (meters)

    @rtype: array of floats(2)
    @return: latitude of new target, longitude of new target
    """
    def _MeterstoGPS(self, north_displacement, east_displacement):
        # Earth’s radius, sphere
        R = 6378137
        # Coordinate offsets in radians
        dLat = north_displacement/R    # Latitude has a relatively constant difference
        dLon = east_displacement/(R*math.cos(math.radians(self.lat_gnd)))  # Longitudinal difference varies by distance from poles
        # OffsetPosition, decimal degrees
        latO = self.lat_gnd + dLat * 180/math.pi
        lonO = self.lon_gnd + dLon * 180/math.pi
        returnvals = [latO, lonO]
        return returnvals
   
   
    def calculate_geolocation(self, mav_lat, mav_lon, height, roll, pitch, yaw, topLeftX, topLeftY, bottomRightX, bottomRightY):
        '''
        Takes in the plane location, angles, and the corners of a cropped region of an image
        Estimates the coordinates for the center of the cropped region
        Based off the algorithm in the textbook provided by Dr. McLain and Dr. Beard

        @type mav_lat: float 
        @param mav_lat: 

        @type mav_lon: float
        @param mav_lon:  

        @type height:
        @param height: The height of the mav (from sea level ?) in m?

        @type roll:
        @param roll: The roll angle of the plane (rotation around the i (north) axis)

        @type pitch:
        @param pitch: The pitch angle of the plane (rotation around the j (east) axis)

        @type yaw:
        @param yaw: The yaw angle of the plane (rotation around the k (down) axis)

        @type topLeftX:
        @param topLeftX: X coordinate of the top left corner of the crop region of the picture

        @type topLefty:
        @param topLefty: Y coordinate of the top left corner of the crop region of the picture

        @type bottomRightX:
        @param bottomRightX: X coordinate of the bottom right corner of the crop region

        @type bottomRightY:
        @param bottomRightY: Y coordinate of the bottom right corner of the crop region

        The data below will be pulled from the database:
        Attitude
        MAV Coordinates
        Pixel Coordinates of Target
        '''

        self.lat_mav, self.lon_mav = mav_lat, mav_lon
        self.phi_deg, self.theta_deg, self.psi_deg = roll, pitch, yaw

        self.TopLeftX = topLeftX
        self.TopLeftY = topLeftY
        self.BottomRightX = bottomRightX
        self.BottomRightY = bottomRightY

        self.height = height

        ''' If pulling from GPS, need this line. Once updated to pull from state, this can removed '''
        # self.height = height - 1387.0 # Account for the elevation above ground level TODO find a safer way to implement this
        # if self.height < 0.0:
        #     print("ERROR in Geolocation.py! height < 0.0")

        # TODO: Account for angle bias
        alpha_az = 0 # Assuming the camera is angled with the top facing out the nose
        alpha_el = math.radians(self.theta_deg) - math.pi/2 # Camera is always (theoretically) -90* in BODY frame, but needs to be in inertial frame

        MaxX = 2000 # Max x pixels
        MaxY = 2000 # Max y pixels
        '''
        The top left and bottom right coordinates of the cropped
        photo are provided. This section finds the center of the
        cropped image and translates it into
        '''
        CenterX = self.BottomRightX - self.TopLeftX
        CenterY = self.BottomRightY - self.TopLeftY
        self.AdjustedCenterX = CenterX-(MaxX/2)
        self.AdjustedCenterY = (-1)*(CenterY-(MaxY/2))

        # TODO: Why is this 4000?
        M = 4000                   # Width of square pixel array (in pixels)
        Ex = self.AdjustedCenterX  # Pixel location of object   
        Ey = -self.AdjustedCenterY # Pixel location of object 

        # TODO: Verify these numbers
        fov_ang = 0.8901179185171081         # field of View angle --> A6000 83* - 32* (in radians)
        f = M/(2.0*math.tan(fov_ang/2.0))   # Focal length in pixels (Textbook Eq. 13.5, pg 231)

        # Unit direction vector to target (l^c / L), Textbook eq 13.9 (pg 229)
        #       Also listed as l^c_d later on 
        l_cusp_c = (1.0/math.sqrt(Ex**2 + Ey**2 + f**2)) * np.array([[Ex],[Ey],[f]])
        '''
        Convert Roll, Pitch and Yaw to radians
        '''
        phi   = math.radians(self.phi_deg)
        theta = math.radians(self.theta_deg) 
        psi   = math.radians(self.psi_deg) 

        '''
        k unit vector in the inertial frame
        '''
        k_i = np.array([[0],[0],[1]])

        '''
        Calculates distance between ground station and mav in meters to determine MAV's
        relative location. Then creates the position vector [Pn Pe Pd]^T
        '''

        # Feb 19, 2020 - Implementing geodesic
        ''' Get the distance between groundstation and mav in meters '''
        average_lat = (self.lat_gnd + self.lat_mav) / 2.0
        average_lon = (self.lon_gnd + self.lon_mav) / 2.0
        estimated_distance_north = self.geod.Inverse(self.lat_gnd, average_lon, self.lat_mav, average_lon, Geodesic.STANDARD)
        estimated_distance_east = self.geod.Inverse(average_lat, self.lon_gnd, average_lat, self.lon_mav, Geodesic.STANDARD)

        # P_i_mav = [Pn, Pe, Pd]
        P_i_mav = np.array([ [estimated_distance_north['s12']], [estimated_distance_east['s12']], [-height]])


        ''' Get the distance from MAV to target in meters'''

        '''
        Trigonometry calculated one time to decrease run time
        '''
        cphi = math.cos(phi)
        sphi = math.sin(phi)
        ctheta = math.cos(theta)
        stheta = math.sin(theta)
        cpsi = math.cos(psi)
        spsi = math.sin(psi)
        caz = math.cos(alpha_az)
        saz= math.sin(alpha_az)
        cel = math.cos(alpha_el) # when alpha_el = 0, this returns 6.123234e-17 instead of 0
        sel = math.sin(alpha_el)

        '''
        Rotation from body to inertial frame
        Found on page 16 of Small Unmanned Aircraft
        '''
        R_v2b = np.array([  [ctheta*cpsi,      ctheta*spsi,     -1*stheta],
                            [(sphi*stheta*cpsi)-(cphi*spsi), (sphi*stheta*spsi)+(cphi*cpsi), sphi*ctheta],
                            [cphi*stheta*cpsi+sphi*spsi, cphi*stheta*spsi-sphi*cpsi, cphi*ctheta]])
        R_b2v = np.transpose(R_v2b)
        R_b2i = R_b2v

        '''
        R_g_to_b
        Found on page 230 of Small Unmanned Aircraft
        '''
        R_b2g1 = np.array([[caz, saz, 0.],[-saz, caz, 0.],[0., 0., 1.]])
        # TODO: The book + paper disagree about this... RH vs LH?
        R_g12g = np.array([[cel, 0., -sel],[0., 1., 0.],[sel, 0., cel]]) # Looks like the transpose of the paper
        # R_g12g = np.transpose(R_g12g)  #
        R_b2g = np.matmul(R_g12g, R_b2g1)
        R_g2b = np.transpose(R_b2g)

        '''
        R_c_to_g
        % Found on page 231 of Small Unmanned Aircraft
        '''
        R_g2c = np.array([[0, 1, 0],[0, 0, 1],[1, 0, 0]])
        R_c2g = np.transpose(R_g2c)

        '''
        % For simplicity, the three Rotation matrices are combined into one below
        RbiRbgRcg = R_b_to_i * R_g_to_b * R_c_to_g, forming Rc2i
        '''
        RbiRbgRcg = np.matmul(np.matmul(R_b2i, R_g2b), R_c2g)

        l_cusp_i = np.matmul(RbiRbgRcg, l_cusp_c)
        P_i_tar = P_i_mav + height*l_cusp_i/((k_i * l_cusp_i)[2])  # Index because k_i * l_cusp will be [0, 0, something]

        #### TODO: Determine if this is good
        ''' Feb 20 2020 - Geographic.lib '''
        ''' Use distance from groundstation to target to get target GPS'''
        eastDist, northDist = P_i_tar[1], P_i_tar[0]
        totalDist = math.sqrt( (eastDist**2) + (northDist**2) )
        aziEst = math.degrees(math.atan2(eastDist, northDist))

        geod_direct = self.geod.Direct(self.lat_gnd, self.lon_gnd, aziEst, totalDist, Geodesic.STANDARD)
        TargetCoordinates = np.array([geod_direct['lat2'], geod_direct['lon2']])
        #####################################

        # print("Target coordinates")
        # print(str(float(TargetCoordinates[0])) + " " + str(float(TargetCoordinates[1])))

        # TODO: See if this works better... (Just returning the MAV location)
        # return float(self.lat_mav), float(self.lon_mav)
        return float(TargetCoordinates[0]), float(TargetCoordinates[1])