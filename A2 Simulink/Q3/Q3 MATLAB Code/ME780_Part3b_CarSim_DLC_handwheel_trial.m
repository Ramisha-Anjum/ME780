T = readtable('CarSim_DLC_input.csv');

% Initial calibration only:
% 2.5 deg road-wheel amplitude becomes 40 deg handwheel amplitude
T.HandwheelAngle_deg = ...
    (40/2.5)*T.RoadWheelAngle_deg;

CarSimSteering = T(:, ...
    {'Time_s','HandwheelAngle_deg'});

writetable(CarSimSteering, ...
    'CarSim_DLC_handwheel_trial.csv');