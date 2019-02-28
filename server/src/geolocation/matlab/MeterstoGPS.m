function returnvals = MeterstoGPS(Lat, Lon, north_displacement, east_displacement)
%  Earth’s radius, sphere
R = 6378137;

% Coordinate offsets in radians
dLat = north_displacement/R;
dLon = east_displacement/(R*cos(pi*Lat/180));

% OffsetPosition, decimal degrees
latO = Lat + dLat * 180/pi;
lonO = Lon + dLon * 180/pi;

returnvals = [latO lonO];
end

