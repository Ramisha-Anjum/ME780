%% setupProjectCarSimCurvedLane
% Initializes the AFS-versus-TV project for the CarSim curved-lane case.

afsProjectRoot = ...
    "\\ecfile1.uwaterloo.ca\r3anjum\My Documents\ME780 Project\" + ...
    "AFS_vs_TV-main";

carSimRoot = ...
    "C:\Users\r3anjum\Documents\Carsim\ME780_Project";

%% Verify folders

assert(isfolder(afsProjectRoot), ...
    "The AFS_vs_TV project folder was not found.");

assert(isfolder(carSimRoot), ...
    "The local CarSim database folder was not found.");

%% Run the main project setup

addpath(afsProjectRoot, "-begin");

projectSetupFile = fullfile(afsProjectRoot, "setupProject.m");

assert(isfile(projectSetupFile), ...
    "setupProject.m was not found.");

run(projectSetupFile);

%% Select curved-lane scenario

scenario = curvedLaneParams();

%% Select initial controller

controllerMode = CTRL_BASELINE;

%% Verify scenario definition

assert(abs(scenario.Vx - 60/3.6) < 1e-6, ...
    "Curved-lane speed must be 60 km/h.");

assert(abs(scenario.mu - 0.90) < 1e-6, ...
    "Curved-lane friction must be 0.90.");

assert(abs(scenario.radius - 100) < 1e-6, ...
    "Curved-lane radius must be 100 m.");

assert(abs(scenario.startTime - 2) < 1e-6, ...
    "The curve must begin at 2 seconds.");

%% Return to CarSim folder

cd(carSimRoot);

%% Open the model

modelName = "experimentHarnessCurvedLane_CarSim_Interface";

modelFile = fullfile( ...
    afsProjectRoot, ...
    "models", ...
    "harness", ...
    modelName + ".slx");

assert(isfile(modelFile), ...
    "The curved-lane CarSim Simulink model was not found.");

open_system(modelFile);

set_param(modelName, "StopTime", "10");
set_param(modelName, "SimulationCommand", "update");

fprintf("\nCurved-lane CarSim setup complete.\n");
fprintf("Project folder: %s\n", afsProjectRoot);
fprintf("CarSim folder:  %s\n", pwd);
fprintf("Speed:          %.3f m/s\n", scenario.Vx);
fprintf("Radius:         %.1f m\n", scenario.radius);
fprintf("Friction:       %.2f\n", scenario.mu);