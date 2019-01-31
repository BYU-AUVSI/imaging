clc
clear

LatStarting = 40.248459;
LonStarting= -111.645809;

LatIntersection = 40.248505;
LonIntersection = -111.643129;

LatRCP = 40.267367;
LonRCP = -111.633662;

Church45Lat = 40.260518;
Church45Lon = -111.630971;

LatYTrailhead = 40.225101;
LonYTrailhead = -111.627675;

LatPark = 40.233062;
LonPark = -111.668215;

LatBamBam = 40.265833;
LonBamBam = -111.681711;

data = GPStoMeters(LatStarting, LonStarting, LatBamBam, LonBamBam);
data2 = MeterstoGPS(LatStarting, LonStarting, data(1), data(2));
