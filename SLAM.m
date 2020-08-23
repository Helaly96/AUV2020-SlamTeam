%------------------------------ SLAM node --------------------------------%
function [state,cov] = SLAM(depth_init,u,PS_reading,delta,LANDMARKS) 
INF = 1000000000;
% Function takes Inputs, Observations and Sampling time of SLAM

% Inputs 
% Past Acceleration in x direction
% Past Acceleration in y direction
% Past Acceleration in z direction 
% Yaw differential (current yaw - old yaw)

% Define your static variables(previous state mean and previous covariance)
persistent state_prev;
if isempty(state_prev)
  state_prev = [0;0;0;0;0;0;0;0;0;depth_init]; % Previous state vector (10 states)
 end
persistent cov_prev;
if isempty(cov_prev)
  cov_prev = [0 0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0 0;
      0 0 0 INF 0 0 0 0 0 0;
      0 0 0 0 INF 0 0 0 0 0;
      0 0 0 0 0 INF 0 0 0 0;
      0 0 0 0 0 0 INF 0 0 0;
      0 0 0 0 0 0 0 INF 0 0;
      0 0 0 0 0 0 0 0 INF 0;
      0 0 0 0 0 0 0 0 0 0]; % Previous covariance vector (10*10)
end
 
% Obtain predicted mean using dynamic model
state = state_prev + [u(1,1)*delta;
    u(2,1)*delta;
    u(3,1)*delta;
    state_prev(5,1)*u(4,1) - state_prev(1,1)*delta;
    -state_prev(4,1)*u(4,1) - state_prev(2,1)*delta;
    -state_prev(3,1)*delta;
    state_prev(8,1)*u(4,1) - state_prev(1,1)*delta;
    -state_prev(7,1)*u(4,1) - state_prev(2,1)*delta;
    -state_prev(3,1)*delta;
    state_prev(3,1)*delta];

% Compute the jacobian
G = [1 0 0 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0 0;
    -delta 0 0 1 u(4,1) 0 0 0 0 0;
    0 -delta 0 u(4,1) 1 0 0 0 0 0;
    0 0 -delta 0 0 1 0 0 0 0;
    -delta 0 0 0 0 0 1 u(4,1) 0 0;
    0 -delta 0 0 0 0 u(4,1) 1 0 0;
    0 0 -delta 0 0 0 0 0 1 0;
    0 0 delta 0 0 0 0 0 0 1]; % Dynamic model jacobian

% Compute the predicted covariance
R = [10^-6 0 0 0 0 0 0 0 0 0;
    0 10^-6 0 0 0 0 0 0 0 0;
    0 0 10^-6 0 0 0 0 0 0 0;
    0 0 0 10^-6 0 0 0 0 0 0;
    0 0 0 0 10^-6 0 0 0 0 0;
    0 0 0 0 0 10^-6 0 0 0 0;
    0 0 0 0 0 0 10^-6 0 0 0;
    0 0 0 0 0 0 0 10^-6 0 0;
    0 0 0 0 0 0 0 0 10^-6 0;
    0 0 0 0 0 0 0 0 0 10^-6]; % Process noise covariance

cov = G*cov_prev*transpose(G) + R; % Predicted Covariance

% Compute the observation jacobian
H = [1 0 0 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0 0;
    0 0 0 1 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0;
    0 0 0 0 0 0 1 0 0 0;
    0 0 0 0 0 0 0 1 0 0
    0 0 0 0 0 0 0 0 1 0
    0 0 0 0 0 0 0 0 0 1]; % Observation jacobian

% Compute the noise matrix & Record readings
Reading = zeros(10,1); % Readings from sensor matrix
%---------------------------------------------------------------------------------------------------------%
% Since we don't have an actual velocity sensor, we'll assume we have a very bad sensor with huge variance%
%---------------------------------------------------------------------------------------------------------%
var_L1vx = INF; % Varriance of velocity reading in x
var_L1vy = INF; % Varriance of velocity reading in y
var_L1vz = INF; % Varriance of velocity reading in z
Reading(1,1) = state(1,1); % Arbitrary x velocity reading from very bad sensor 
Reading(2,1) = state(2,1); % Arbitrary y velocity reading from very bad sensor
Reading(3,1) = state(3,1); % Arbitrary y velocity reading from very bad sensor
Reading(10,1) = PS_reading; % Pressure sensor reading
var_absz = 1.778*10^-4; % Pressure sensor variance
% Our update parameters will depend on whether we have a clear visual
if(LANDMARKS(1,1,1) == 'n') % If first landmark is not detected
    var_L1x = INF;
    var_L1y = INF;
    var_L1z = INF;
    Reading(4,1) = state(4,1);
    Reading(5,1) = state(5,1);
    Reading(6,1) = state(6,1);
elseif(LANDMARKS(1,1,1) == 'd') % If first landmark is detected
    var_L1x = 4.44*10^-3; % Variance of zed camera reading (assumed to be +/-20cm error)
    var_L1y = 4.44*10^-3;
    var_L1z = 4.44*10^-3;
    Reading(4,1) = LANDMARKS(1,2,1); % Actual position readings from camera
    Reading(5,1) = LANDMARKS(1,3,1);
    Reading(6,1) = LANDMARKS(1,4,1);
end
% Second Landmark
if(LANDMARKS(2,1,1) == 'n') % If second landmark is not detected
    var_L2x = INF;
    var_L2y = INF;
    var_L2z = INF;
    Reading(7,1) = state(7,1);
    Reading(8,1) = state(8,1);
    Reading(9,1) = state(9,1);
elseif(LANDMARKS(2,1,1) == 'd') % If second landmark is detected
    var_L2x = 4.44*10^-3; % Variance of zed camera reading (assumed to be +/-20cm error)
    var_L2y = 4.44*10^-3;
    var_L2z = 4.44*10^-3;
    Reading(7,1) = LANDMARKS(2,2,1); % Actual position readings from camera
    Reading(8,1) = LANDMARKS(2,3,1);
    Reading(9,1) = LANDMARKS(2,4,1);
end

Q = [var_L1vx 0 0 0 0 0 0 0 0 0;
    0 var_L1vy 0 0 0 0 0 0 0 0;
    0 0 var_L1vz 0 0 0 0 0 0 0;
    0 0 0 var_L1x 0 0 0 0 0 0;
    0 0 0 0 var_L1y 0 0 0 0 0;
    0 0 0 0 0 var_L1z 0 0 0 0;
    0 0 0 0 0 0 var_L2x 0 0 0;
    0 0 0 0 0 0 0 var_L2y 0 0;
    0 0 0 0 0 0 0 0 var_L2z 0;
    0 0 0 0 0 0 0 0 0 var_absz]; % Measurement noise matrix

% Compute kalman gain 
K = cov*transpose(H)*inv(H*cov*transpose(H) + Q); % Kalman gain

% Compute updated state and covariance
state = state + K*(Reading-state);
cov = (eye(10)-K*H)*cov;
state_prev = state;
cov_prev = cov;
end