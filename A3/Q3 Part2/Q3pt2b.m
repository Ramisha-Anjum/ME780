%% ME780 Assignment 3 part 2 b)
clear; clc; close all;

L = 2.6;               % Wheelbase [m]
K_us = 0.001538;       % Understeer coefficient [s^2/m]

U = linspace(0,60,500);    % Vehicle speed [m/s]

G_r = U./(L + K_us*U.^2);  % Yaw-rate gain [1/s]

plot(U,G_r,'b','LineWidth',2);
grid on;
box on;

xlabel('Vehicle speed, U [m/s]');
ylabel('Yaw-rate gain, r_{ss}/\delta [s^{-1}]');
title('Steady-State Yaw-Rate Gain');