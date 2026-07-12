clear;
clc;

dt = 0.01;
t = (0:dt:12)';

delta_deg = zeros(size(t));

% First sine pulse: 1 to 5 s
idx1 = t >= 1.0 & t < 5.0;
delta_deg(idx1) = ...
    2.5*sin(2*pi*0.25*(t(idx1)-1.0));

% Second opposite sine pulse: 5.5 to 9.5 s
idx2 = t >= 5.5 & t < 9.5;
delta_deg(idx2) = ...
   -2.5*sin(2*pi*0.25*(t(idx2)-5.5));

CarSimInput = table(t,delta_deg, ...
    'VariableNames',{'Time_s','RoadWheelAngle_deg'});

writetable(CarSimInput,'CarSim_DLC_input.csv');