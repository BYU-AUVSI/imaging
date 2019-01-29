clear
clc
digits(100)
Coordinates % Load in the Test GPS Points
%% Initial Parameters
% These initial parameters represent data that will be readily accessible
% on the MAV. Pixel Array, Field of View angle are specifications of the
% camera, and the normalized line of sight vector (l_cusp_c)  will be
% determined by either the autonomous detection program, or the manual
% interface clients.

%TEST DATA
MAV_HEIGHT = 16;
X_PIXEL = -15;
Y_PIXEL = 1028;
ROLL = 0;
PITCH = 0.523599;
YAW = pi/2;
MAV_Coordinates = C.skybridge;
GroundStation_Coordinates = C.TestGroundStation;

M = 4000; %square pixel array (width and height if square)
Ex = X_PIXEL; %x_pixel/M; %X pixels, normalized
Ey = (-1)*Y_PIXEL; %y_pixel/M; %Y pixels, normalized
fov_ang = 0.872665;%sym('fov_ang'); %field of View angle --> A6000 83* - 32* (in radians)
f = M/(2*tan(fov_ang/2)); %focal length  in units of pixels

l_cusp_c = 1/sqrt(Ex^2 + Ey^2 + f^2) * [Ex;Ey;f];

roll = ROLL; %sym('roll'); %Radians
pitch = PITCH;%-0.523599; %sym('pitch'); %RadiansP
yaw = YAW;%pi/2; %sym('yaw'); %Radians

alpha_az = 0;%sym('az'); %Azmuth Angle: Should equal yaw.
alpha_el = -pi/2 + pitch;%sym('el'); %Elevation Angle: "Pitch" of the gimbal

k_i = [0;0;1];

MavGPSData = [GroundStation_Coordinates(1), GroundStation_Coordinates(2), MAV_Coordinates(1), MAV_Coordinates(2)];
MavLocationData = GPStoMeters(MavGPSData(1), MavGPSData(2), MavGPSData(3), MavGPSData(4));

Pn = MavLocationData(1);
Pe = MavLocationData(2);
Pd = (-1)*MAV_HEIGHT;%sym('Pd');
h = -Pd;%;sym('h');
%% Rotation Matrices
% The location of the target is defined in the camera coordinate frame
% system, and must be transformed into the inertial frame to provide GPS
% coordinates. This transformation is done using three rotation matrices:
% Moving Camera frame to Gimbal coordinates; Gimbal coordinates to body
% frame; body frame to inertial frame

% R_b_to_i
% Found on page 15 of Small Unmanned Aircraft

R_v_to_b = [cos(pitch)*cos(yaw) cos(pitch)*sin(yaw) -sin(pitch); ...
    sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw) ...
    sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw) sin(roll)*cos(pitch); ...
    cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw) ...
    cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw) cos(roll)*cos(pitch)];

R_b_to_v = R_v_to_b'; %R_v_b is a translation of R_i_b. It may work, but I may need one more step.
R_b_to_i = R_b_to_v;

% ------------------------------------------------------------------------
% R_g_to_b
% Found on page 227 of Small Unmanned Aircraft
R_b_to_g1 = [cos(alpha_az) sin(alpha_az) 0;...
    -sin(alpha_az) cos(alpha_az) 0;...
    0 0 1];

R_g1_to_g = [cos(alpha_el) 0 -sin(alpha_el);...
    0 1 0;...
    sin(alpha_el) 0 cos(alpha_el)];

R_b_to_g = R_g1_to_g*R_b_to_g1;
R_g_to_b = R_b_to_g';

% ------------------------------------------------------------------------
% R_c_to_g
% Found on page 227 of Small Unmanned Aircraft
R_g_to_c = [0 1 0;...
    0 0 1;...
    1 0 0];
R_c_to_g = R_g_to_c';

% For simplicity, the three Rotation matrices are combined into one below
RbiRbgRcg = R_b_to_i * R_g_to_b * R_c_to_g;

%% Geolocation Algorithm
% The algorithm uses the line of sight vector in the camera frame and
% rotates it into the inertial frame. There, it is multiplied by a scaler
% (height) and divided by the projection of the transformed line of sight
% vector onto the unit vector k_i in the inertial frame
P_i_mav = [Pn;Pe;Pd];
P_i_obj = P_i_mav + h*(RbiRbgRcg*l_cusp_c)/(dot(k_i,(RbiRbgRcg*l_cusp_c)));

%% Publish
% The coordinates of the groundstation are used as a reference for
% determining the relative position of the MAV, which is in turn used to
% find the relative position of the target. The coordinates for all three
% points are shown below.
disp("Groundstation Coordinates ");
fprintf(MavGPSData(1) + " " + MavGPSData(2) + "\n\n");

disp("MAV Coordinates ");
fprintf(MavGPSData(3) + " " + MavGPSData(4) + "\n\n");

TargetCoordinates = MeterstoGPS(LatTestGroundStation, LonTestGroundStation, P_i_obj(1), P_i_obj(2));
disp("Target Coordinates ");
fprintf(TargetCoordinates(1)+", "+TargetCoordinates(2) + "\n");