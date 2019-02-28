function returnvalues = GPStoMeters( lat1, lon1, lat2, lon2 )

d2r = 0.0174532925199433;

dlong = (lon2 - lon1) * d2r;
dlat = (lat2 - lat1) * d2r;
a = sin(dlat/2.0)^2 + cos(lat1*d2r) * cos(lat2*d2r) * sin(dlong/2.0)^2;
c = 2 * atan2(sqrt(a), sqrt(1-a));
d = 6367 * c; %Distance between points in meters
d_meters = d*1000;

dy = lat2 - lat1;
dx = cos(d2r*lat1)*(lon2 - lon1);
angle = atan2(dy, dx);
east_dis_meters = d_meters*cos(angle);
north_dis_meters = d_meters*sin(angle);
returnvalues = [north_dis_meters, east_dis_meters, d_meters, angle];
end

