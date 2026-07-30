function scenario = DLCParams()
%DLCParams Dry-road double-lane-change scenario.
%
% The maneuver moves the reference path from the original lane to an
% adjacent lane and then returns to the original lane.
%
% The path is spatially defined as a function of longitudinal position X.
% This makes it usable with both the bicycle plant and later with CarSim,
% even when actual vehicle speed is not perfectly constant.

scenario.name = "Double Lane Change";

%% Required project conditions

scenario.Vx = 70/3.6;      % 70 km/h [m/s]
scenario.mu = 0.9;         % dry-road friction coefficient

%% Path geometry

scenario.laneWidth = 3.5;         % lateral lane shift [m]
scenario.xStart = 30;             % straight lead-in distance [m]
scenario.transitionLength = 45;   % length of each lane transition [m]
scenario.holdLength = 25;         % distance in adjacent lane [m]

% Important longitudinal locations.
scenario.xFirstEnd = ...
    scenario.xStart + scenario.transitionLength;

scenario.xReturnStart = ...
    scenario.xFirstEnd + scenario.holdLength;

scenario.xEnd = ...
    scenario.xReturnStart + scenario.transitionLength;

%% Simulation timing

% Approximate time at which the first lane change begins.
scenario.startTime = scenario.xStart/scenario.Vx;

% Approximate time at which the vehicle returns to the original lane.
scenario.endTime = scenario.xEnd/scenario.Vx;

% Allow enough time after the maneuver for the vehicle to settle.
scenario.settlingDistance = 20;   % [m]

scenario.stopTime = ...
    (scenario.xEnd + scenario.settlingDistance)/scenario.Vx + 1.0;

%% Metric-evaluation interval

% Start metrics at the beginning of the maneuver. Do not include the
% initial straight segment because it would artificially reduce RMS values.
scenario.evaluationStart = scenario.startTime;

% Include approximately 20 m of post-maneuver settling.
scenario.evaluationEnd = ...
    (scenario.xEnd + scenario.settlingDistance)/scenario.Vx;

end