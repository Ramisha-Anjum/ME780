%% ME780 Assignment 2 - Part 3(b)
% Linear single-track (bicycle) model: double lane change at 35 km/h
% Run this script before opening the Simulink model.
%
% Steering input:
%   Pulse 1: +sinusoid from 1.0 to 5.0 s
%   Pulse 2: -sinusoid from 5.5 to 9.5 s
%   Front road-wheel amplitude = 2.5 deg
%   Frequency = 0.25 Hz
%
% This produces an approximately 3.5 m lane displacement while keeping
% sideslip and lateral acceleration within the linear-model range.

clear; clc; close all;

%% Vehicle parameters
m  = 1919.2;          % vehicle mass [kg]
Iz = 2687.1;          % yaw inertia [kg m^2]
a  = 1.180;           % CG to front axle [m]
b  = 1.770;           % CG to rear axle [m]
L  = a + b;           % wheelbase [m]
u  = 35/3.6;          % constant longitudinal speed [m/s]

Caf = 143566.4;       % equivalent front axle cornering stiffness [N/rad]
Car = 97615.6;        % equivalent rear axle cornering stiffness [N/rad]

%% Two-state bicycle model: x = [v; r]
A = [-(Caf+Car)/(m*u), ...
     -((a*Caf-b*Car)/(m*u) + u);
     -(a*Caf-b*Car)/(Iz*u), ...
     -(a^2*Caf+b^2*Car)/(Iz*u)];

B = [Caf/m;
     a*Caf/Iz];

%% Augmented model: xa = [v; r; psi; Y]
% psi_dot = r
% Y_dot   = v + u*psi  (small-angle kinematics)
A_aug = [A(1,1), A(1,2), 0, 0;
         A(2,1), A(2,2), 0, 0;
         0,      1,      0, 0;
         1,      0,      u, 0];

B_aug = [B;
         0;
         0];

%% Output equations
% Outputs: [v, r, ay, beta, psi, Y]
C_aug = [1,       0,          0, 0;        % lateral velocity
         0,       1,          0, 0;        % yaw rate
         A(1,1), A(1,2)+u,    0, 0;        % lateral acceleration
        -1/u,     0,          0, 0;        % sideslip angle
         0,       0,          1, 0;        % yaw angle
         0,       0,          0, 1];       % lateral displacement

D_aug = [0;
         0;
         B(1);
         0;
         0;
         0];

sys_bicycle = ss(A_aug,B_aug,C_aug,D_aug);

%% Double lane-change steering input
dt = 0.001;
tEnd = 12;
t = (0:dt:tEnd)';

deltaMax_deg = 2.5;
deltaMax = deg2rad(deltaMax_deg);
f = 0.25;            % Hz

delta = zeros(size(t));

idx1 = t >= 1.0 & t < 5.0;
delta(idx1) = deltaMax*sin(2*pi*f*(t(idx1)-1.0));

idx2 = t >= 5.5 & t < 9.5;
delta(idx2) = -deltaMax*sin(2*pi*f*(t(idx2)-5.5));

% Workspace formats for Simulink
delta_ts = timeseries(delta,t);
delta_deg_ts = timeseries( ...
    rad2deg(delta_ts.Data), ...
    delta_ts.Time);
u_ts = timeseries(u*ones(size(t)),t);

save('DLC_input.mat','delta_ts','delta_deg_ts','u_ts','t','delta','u');

%% MATLAB simulation
[y,tOut,x] = lsim(sys_bicycle,delta,t);

v    = y(:,1);
r    = y(:,2);
ay   = y(:,3);
beta = y(:,4);
psi  = y(:,5);
Y    = y(:,6);
X    = u*tOut;        % adequate at small sideslip/yaw for comparison

%% Key results
fprintf('\nME780 Part 3(b): Linear Bicycle Model\n');
fprintf('Longitudinal speed: %.4f m/s (35 km/h)\n',u);
fprintf('Steering amplitude: %.2f deg\n',deltaMax_deg);
fprintf('Steering frequency: %.2f Hz\n',f);
fprintf('Peak |yaw rate|: %.5f rad/s (%.3f deg/s)\n', ...
    max(abs(r)),rad2deg(max(abs(r))));
fprintf('Peak |lateral acceleration|: %.5f m/s^2 (%.4f g)\n', ...
    max(abs(ay)),max(abs(ay))/9.81);
fprintf('Peak |sideslip|: %.5f rad (%.3f deg)\n', ...
    max(abs(beta)),rad2deg(max(abs(beta))));
fprintf('Maximum lateral displacement: %.4f m\n',max(Y));
fprintf('Final lateral displacement: %.6f m\n',Y(end));

%% Plots
figure('Name','ME780 Part 3(b) - Bicycle Model');
tiledlayout(3,2);

nexttile;
plot(tOut,rad2deg(delta),'LineWidth',1.4);
grid on;
xlabel('Time [s]');
ylabel('\delta [deg]');
title('Front Road-Wheel Steering Input');

nexttile;
plot(tOut,r,'LineWidth',1.4);
grid on;
xlabel('Time [s]');
ylabel('Yaw rate [rad/s]');
title('Yaw Rate');

nexttile;
plot(tOut,ay,'LineWidth',1.4);
grid on;
xlabel('Time [s]');
ylabel('a_y [m/s^2]');
title('Lateral Acceleration');

nexttile;
plot(tOut,rad2deg(beta),'LineWidth',1.4);
grid on;
xlabel('Time [s]');
ylabel('\beta [deg]');
title('Vehicle Sideslip Angle');

nexttile;
plot(tOut,Y,'LineWidth',1.4);
grid on;
xlabel('Time [s]');
ylabel('Y [m]');
title('Lateral Displacement');

nexttile;
plot(X,Y,'LineWidth',1.4);
grid on;
xlabel('X [m]');
ylabel('Y [m]');
title('Vehicle Trajectory');

%% Save outputs for later comparison
Steer_deg = rad2deg(delta);
results = table(tOut,delta,Steer_deg,v,r,ay,beta,psi,X,Y, ...
    'VariableNames',{'Time_s','Steer_rad','Steer_deg','LateralVelocity_mps', ...
    'YawRate_radps','LateralAcceleration_mps2','Sideslip_rad', ...
    'YawAngle_rad','X_m','Y_m'});

writetable(results,'bicycle_results.csv');
save('bicycle_results.mat','results','A','B','A_aug','B_aug', ...
     'C_aug','D_aug','sys_bicycle');
