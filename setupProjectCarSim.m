%% setupProjectCarSim
% Initialize the AFS-vs-TV project while keeping MATLAB in the CarSim
% database folder that contains simfile.sim.

carSimRoot = ...
    "C:\Users\r3anjum\Documents\Carsim\ME780_Project";

afsProjectRoot = ...
    "\\ecfile1.uwaterloo.ca\r3anjum\My Documents\ME780 Project\AFS_vs_TV-main";

%% Validate the CarSim database

if ~isfolder(carSimRoot)
    error( ...
        "CarSim database folder was not found:\n%s", ...
        carSimRoot);
end

simfilePath = fullfile(carSimRoot, "simfile.sim");

if ~isfile(simfilePath)
    error( ...
        "simfile.sim was not found:\n%s\n\n" + ...
        "Open the CarSim run and use Send to Simulink first.", ...
        simfilePath);
end

cd(carSimRoot);

%% Initialize the permanent MATLAB project

projectSetupFile = fullfile(afsProjectRoot, "setupProject.m");

if ~isfile(projectSetupFile)
    error( ...
        "setupProject.m was not found:\n%s", ...
        projectSetupFile);
end

addpath(afsProjectRoot, "-begin");

clear setupProject
rehash path

run(projectSetupFile);

% setupProject should not control the CarSim working directory.
cd(carSimRoot);

%% DLC controller selection

scenario = DLCParams();
% scenario = curvedLaneParams();

controllerMode = CTRL_BASELINE;
% controllerMode = CTRL_AFS;
% controllerMode = CTRL_TV;

fprintf("\nCarSim project initialized successfully.\n");
fprintf("Current folder:\n  %s\n", pwd);
fprintf("Simfile:\n  %s\n", simfilePath);
fprintf("MATLAB project:\n  %s\n", afsProjectRoot);
fprintf("Scenario: DLC at %.1f km/h, mu = %.2f\n", ...
    scenario.Vx*3.6, scenario.mu);
fprintf("Controller: Baseline\n\n");