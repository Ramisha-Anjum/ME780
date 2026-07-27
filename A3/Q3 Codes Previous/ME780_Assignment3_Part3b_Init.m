clear;
clc;
close all;

%% Vehicle and reference-model parameters
L   = 2.950;               % wheelbase [m]
Kus = 1.22226447e-4;       % understeer coefficient [s^2/m]
g   = 9.81;

%% PI controller
Kp = 28835.61;
Ki = 57671.23;
Kd = 0;

Mz_limit = 2800;           % corrective moment limit [N m]

%% Simulation settings
Vx_target_kph = 60;
Tstop = 12;
Ts = 0.001;

%% Change these before each run
mu_road = 0.90;            % use 0.90 or 0.55
controller_enable = 0;     % 0 = OFF, 1 = ON