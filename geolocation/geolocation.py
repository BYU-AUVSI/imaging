'''
            BYU AUVSI-SUAS Capstone
             Geolocation Algorithm
              Connor Olsen, 2019
'''

import numpy as np
import math as math

def returnvalues = GPStoMeters(lat1, lon1, lat2, lon2)
    d2r = 0.0174532925199433
    dlong = (lon2 - lon1) * d2r
    dlat = (lat2 - lat1) * d2r
    a = sin(dlat/2.0)^2 + cos(lat1*d2r) * cos(lat2*d2r) * sin(dlong/2.0)^2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    d = 6367 * c; #Distance between points in meters
    d_meters = d * 1000
    dy = lat2 - lat1
    dx = cos(d2r * lat1) * (lon2 - lon1)
    angle = atan2(dy, dx)
    east_dis_meters = d_meters * cos(angle)
    north_dis_meters = d_meters * sin(angle)
    returnvalues = np.array([north_dis_meters, east_dis_meters, d_meters, angle])
    return returnvalues

def MeterstoGPS(Lat, Lon, north_displacement, east_displacement)
    # Earth’s radius, sphere
    R = 6378137
    # Coordinate offsets in radians
    dLat = north_displacement/R
    dLon = east_displacement/(R*cos(pi*Lat/180))
    # OffsetPosition, decimal degrees
    latO = Lat + dLat * 180/pi
    lonO = Lon + dLon * 180/pi
    returnvals = [latO lonO];
    return returnvals



print("Geolocation Algorithm")

# The data below will be pulled from the database:
# Attitude
# MAV Coordinates
# Pixel Coordinates of Target

# For now, we will use dummy data
phi = 0
theta = 3.14159/4
psi = 3.14159/2
alpha_az = 0 # Assuming the camera is angled with the top facing out the nose
alpha_el = -pi/2 + theta
lat_mav = 0
lon_mav = 0
height = 100

MaxX = 2000 # Max x pixels
MaxY = 2000 # Max y pixels
# The top left and bottom right coordinates of the cropped
# photo are provided. This section finds the center of the
# cropped image and translates it into
TopLeftX = 0
TopLeftY = 0
BottomRightX = 0
BottomRightY = 0
CenterX = BottomRightX - TopLeftX
CenterY = BottomRightY - TopLeftY
AdjustedCenterX = CenterX-(MaxX/2)
AdjustedCenterY = (-1)*(CenterY-(MaxY/2))

positionData = GPStoMeters(lat_groundstation, lon_groundstation, lat_mav, lon_mav)
P_i_mav = np.array([positionData(0), positionData(1), -height])
cphi = math.cos(phi)
sphi = math.sin(phi)
ctheta = math.cos(theta)
stheta = math.sin(theta)
cpsi = math.cos(psi)
spsi = math.sin(psi)
caz = math.cos(alpha_az)
saz= math.sin(alpha_az)
cel = math.cos(alpha_el)
sel = math.sin(alpha_el)


# Rotation from body to inertial frame
# Found on page 15 of Small Unmanned Aircraft

R_v2b = np.array([[ctheta*cpsi, ctheta*spsi, -stheta],\
[sphi*stheta*cpsi-cphi*spsi, sphi*stheta*spsi+cphi*cpsi, sphi*ctheta],\
[cphi*stheta*cpsi+sphi*spsi, cphi*stheta*spsi-sphi*cpsi, cphi*ctheta]])
R_b2v = np.transpose(R_v2b)
R_b2i = R_b2v

print(R_b2i)

# -----------------------------------------

# R_g_to_b
# Found on page 227 of Small Unmanned Aircraft

R_b2g1 = np.array([[caz, saz, 0],[-saz, saz, 0],[0, 0, 1]])
R_g12g = np.array([[cel, 0, -sel],[0, 1, 0],[sel, 0, cel]])
R_b2g = R_g12g*R_b2g1
R_g2b = np.transpose(R_b2g)
print(R_g2b)

# -----------------------------------------

#  R_c_to_g
# % Found on page 227 of Small Unmanned Aircraft

R_g2c = np.array([[0, 1, 0],[0, 0, 1],[1, 0, 0]])
R_c2g = np.transpose(R_g2c)
print(R_c2g)

# -----------------------------------------

# % For simplicity, the three Rotation matrices are combined into one below
# RbiRbgRcg = R_b_to_i * R_g_to_b * R_c_to_g;
RbiRbgRcg = R_b2i * R_g2b * R_c2g
print(RbiRbgRcg)

# -----------------------------------------
