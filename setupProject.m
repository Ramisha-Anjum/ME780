% projectRoot = fileparts(mfilename("fullpath"));
projectRoot = uigetdir;
cd(projectRoot);

addpath(fullfile(projectRoot, "config", "scenarios"));
addpath(fullfile(projectRoot, "models", "harness"));
addpath(fullfile(projectRoot, "models", "controllers"));
addpath(fullfile(projectRoot, "models", "plants"));
addpath(fullfile(projectRoot, "src"));

VehicleStateBus = stateBus();
ReferenceBus = referenceBus();

run(fullfile(projectRoot, "config", "eClassParams.m"));
run(fullfile(projectRoot, "config", "controllerParams.m"));

scenario = curvedLaneParams();

% controllerMode = CTRL_BASELINE;
controllerMode = CTRL_AFS;
% controllerMode = CTRL_TV;
