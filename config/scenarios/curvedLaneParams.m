function scenario = curvedLaneParams()
%curvedLaneParams Required dry-road curved-lane scenario.

scenario.Vx = 60/3.6;   % 60 km/h [m/s]
scenario.mu = 0.9;      % Dry-road friction coefficient
scenario.radius = 100;  % Curve radius [m] - team-defined value
scenario.startTime = 2; % Initial straight section [s]

end