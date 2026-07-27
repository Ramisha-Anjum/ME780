function results = runCurvedLaneComparisonAll()
%runCurvedLaneComparisonAll Compare Baseline, AFS, and TV on the curved-lane test.
%
% The function runs both controllers through the same experimentHarness
% model, extracts the signals already instrumented in the harness, computes
% the Week 1 comparison metrics, and writes reproducible result artifacts to
% results/week1_curved_lane.
%
% Usage from the project root:
%   setupProject
%   results = runCurvedLaneComparison();

projectRoot = fileparts(fileparts(mfilename("fullpath")));
outputDirectory = fullfile(projectRoot, "results", "curved_lane_all_controllers");
if ~isfolder(outputDirectory)
    mkdir(outputDirectory);
end

originalDirectory = pwd;
directoryCleanup = onCleanup(@() cd(originalDirectory));
cd(projectRoot);

hadControllerMode = evalin("base", "exist('controllerMode','var') == 1");
if hadControllerMode
    originalControllerMode = evalin("base", "controllerMode");
else
    originalControllerMode = [];
end
modeCleanup = onCleanup(@() restoreControllerMode( ...
    hadControllerMode, originalControllerMode));

% Initialize buses, vehicle parameters, controller parameters, and scenario
% in the base workspace used by Simulink.
evalin("base", "setupProject");

scenario = evalin("base", "scenario");
tv = evalin("base", "tv");
afs = evalin("base", "afs");

% Both augmentation controllers currently use the same yaw-reference
% filter time constant. The comparison reference should therefore be
% identical for Baseline, AFS, and TV.
assert(abs(afs.tauRef - tv.tauRef) < 1e-12, ...
    "AFS and TV must use the same reference-filter time constant.");

referenceTau = afs.tauRef;

% Force the online AFS solver to rebuild its persistent MPC matrices.
clear solveAFSMPCOnline

controllerModes = [evalin("base", "CTRL_BASELINE"), ...
                   evalin("base", "CTRL_AFS"), ...
                   evalin("base", "CTRL_TV")];

controllerNames = [
    "Baseline", "AFS", "TV"];
modelName = "experimentHarness";
stopTime = 10;

runData = repmat(emptyRunData(), 1, numel(controllerModes));

for k = 1:numel(controllerModes)
    previousRunIds = Simulink.sdi.getAllRunIDs;

    simInput = Simulink.SimulationInput(modelName);
    simInput = simInput.setVariable("controllerMode", controllerModes(k));
    simInput = simInput.setModelParameter("StopTime", num2str(stopTime));
    % SimulationInput carries both the model name and the per-run variant.
    % Assign the output to avoid polluting the base workspace with `ans`.
    simulationOutput = sim(simInput); %#ok<NASGU>

    currentRunIds = Simulink.sdi.getAllRunIDs;
    newRunIds = setdiff(currentRunIds, previousRunIds, "stable");
    if isempty(newRunIds)
        error("runCurvedLaneComparison:MissingSDIRun", ...
            "Simulation Data Inspector did not record the %s run.", ...
            controllerNames(k));
    end

    sdiRun = Simulink.sdi.getRun(newRunIds(end));
    runData(k) = extractRunData(sdiRun, controllerNames(k), scenario, referenceTau);
end

evaluationStart = scenario.startTime + 1.0;
metrics = calculateMetrics(runData, evaluationStart, tv.MzMax, afs.deltaMax);

% Acceptance invariants come from the model interface and scenario, rather
% than arbitrary performance thresholds.
validateRuns(runData, scenario, tv.MzMax, afs);

writetable(metrics, fullfile(outputDirectory, "metrics.csv"));
createTrackingFigure(runData, evaluationStart, outputDirectory);
createEffortFigure(runData, evaluationStart, outputDirectory);

results = struct;
results.generatedAt = string(datetime("now", "TimeZone", "local"));
results.scenario = scenario;
results.evaluationStart = evaluationStart;
results.metrics = metrics;
results.runs = runData;
results.outputDirectory = string(outputDirectory);
save(fullfile(outputDirectory, "comparison_results.mat"), "results");

fprintf("\nCurved-lane Baseline, AFS, and TV comparison\n");
disp(metrics);
fprintf("Results written to:\n  %s\n", outputDirectory);

% Retain cleanup objects until all output has been written.
clear modeCleanup directoryCleanup
end

function data = extractRunData(sdiRun, controllerName, scenario, referenceTau)
xSignal = requireSignal(sdiRun, "Plant:1.X");
time = double(xSignal.Time(:));

data = emptyRunData();
data.controller = controllerName;
data.time = time;
data.X = sampleSignal(xSignal, time);
data.Y = sampleSignal(requireSignal(sdiRun, "Plant:1.Y"), time);
data.psi = sampleSignal(requireSignal(sdiRun, "Plant:1.psi"), time);
data.Vx = sampleSignal(requireSignal(sdiRun, "Plant:1.Vx"), time);
data.Vy = sampleSignal(requireSignal(sdiRun, "Plant:1.Vy"), time);
data.r = sampleSignal(requireSignal(sdiRun, "Plant:1.r"), time);
data.beta = sampleSignal(requireSignal(sdiRun, "Plant:1.beta"), time);
data.ay = sampleSignal(requireSignal(sdiRun, "Plant:1.ay"), time);

data.XRef = sampleSignal(requireSignal(sdiRun, "Ref Bus:1.X_ref"), time);
data.YRef = sampleSignal(requireSignal(sdiRun, "Ref Bus:1.Y_ref"), time);
data.psiRef = sampleSignal(requireSignal(sdiRun, "Ref Bus:1.psi_ref"), time);
data.kappaRef = sampleSignal( ...
    requireSignal(sdiRun, "Ref Bus:1.kappa_ref"), time);
data.mu = sampleSignal(requireSignal(sdiRun, "Ref Bus:1.mu"), time);

data.deltaBase = sampleSignal( ...
    requireSignal(sdiRun, "BaseSteering:1"), time);
data.deltaF = sampleSignal(requireSignal(sdiRun, "Controller:1"), time);
data.Mz = sampleSignal(requireSignal(sdiRun, "Controller:2"), time);
data.deltaAdd = data.deltaF - data.deltaBase;
data.deltaAddRate = numericalDerivative(time, data.deltaAdd);

speedForLimit = max(abs(data.Vx), 0.1);
yawRateLimit = data.mu .* 9.81 ./ speedForLimit;
data.rTarget = min(max(data.Vx .* data.kappaRef, -yawRateLimit), ...
                   yawRateLimit);
data.rRef = firstOrderResponse(time, data.rTarget, referenceTau);

data.crossTrackError = -sin(data.psiRef) .* (data.X - data.XRef) ...
                     + cos(data.psiRef) .* (data.Y - data.YRef);
data.headingError = atan2(sin(data.psiRef - data.psi), ...
                          cos(data.psiRef - data.psi));
data.yawRateError = data.rRef - data.r;
data.positionError = hypot(data.X - data.XRef, data.Y - data.YRef);

% Preserve the scenario alongside each run so saved results remain
% interpretable even if the configuration file later changes.
data.scenarioSpeed = scenario.Vx;
data.scenarioRadius = scenario.radius;
end

function metrics = calculateMetrics(runData, evaluationStart, mzMaximum, afsMaximum)

nRuns = numel(runData);

controller = strings(nRuns, 1);
finalYawRate = zeros(nRuns, 1);

yawRateRms = zeros(nRuns, 1);
yawRatePeak = zeros(nRuns, 1);

crossTrackRms = zeros(nRuns, 1);
crossTrackPeak = zeros(nRuns, 1);
headingPeakDeg = zeros(nRuns, 1);

betaPeakDeg = zeros(nRuns, 1);
ayPeak = zeros(nRuns, 1);

baseSteeringRmsDeg = zeros(nRuns, 1);
baseSteeringPeakDeg = zeros(nRuns, 1);

additionalSteeringRmsDeg = zeros(nRuns, 1);
additionalSteeringPeakDeg = zeros(nRuns, 1);
additionalSteeringRatePeakDegS = zeros(nRuns, 1);
additionalSteeringSaturationFraction = zeros(nRuns, 1);

totalSteeringRmsDeg = zeros(nRuns, 1);
totalSteeringPeakDeg = zeros(nRuns, 1);

mzRms = zeros(nRuns, 1);
mzPeak = zeros(nRuns, 1);
mzSaturationFraction = zeros(nRuns, 1);

for k = 1:nRuns
    evaluationMask = runData(k).time >= evaluationStart;
    controller(k) = runData(k).controller;
    finalYawRate(k) = runData(k).r(end);
    yawRateRms(k) = rootMeanSquare(runData(k).yawRateError(evaluationMask));
    yawRatePeak(k) = max(abs(runData(k).yawRateError(evaluationMask)));
    crossTrackRms(k) = rootMeanSquare(runData(k).crossTrackError(evaluationMask));
    crossTrackPeak(k) = max(abs(runData(k).crossTrackError(evaluationMask)));
    headingPeakDeg(k) = rad2deg(max(abs(runData(k).headingError(evaluationMask))));
    betaPeakDeg(k) = rad2deg(max(abs(runData(k).beta(evaluationMask))));
    ayPeak(k) = max(abs(runData(k).ay(evaluationMask)));

    % Common baseline lane-controller steering.
    baseSteeringRmsDeg(k) = rad2deg(rootMeanSquare(runData(k).deltaBase(evaluationMask)));
    baseSteeringPeakDeg(k) = rad2deg(max(abs(runData(k).deltaBase(evaluationMask))));

    % Additional steering generated only by AFS.
    additionalSteeringRmsDeg(k) = rad2deg(rootMeanSquare(runData(k).deltaAdd(evaluationMask)));
    additionalSteeringPeakDeg(k) = rad2deg(max(abs(runData(k).deltaAdd(evaluationMask))));
    additionalSteeringRatePeakDegS(k) = rad2deg(max(abs(runData(k).deltaAddRate(evaluationMask))));
    additionalSteeringSaturationFraction(k) = mean( ...
        abs(runData(k).deltaAdd(evaluationMask)) >= 0.99*afsMaximum);

    % Total road-wheel steering sent to the plant.
    totalSteeringRmsDeg(k) = rad2deg(rootMeanSquare(runData(k).deltaF(evaluationMask)));
    totalSteeringPeakDeg(k) = rad2deg(max(abs(runData(k).deltaF(evaluationMask))));

    % Direct yaw moment generated only by TV.
    mzRms(k) = rootMeanSquare(runData(k).Mz(evaluationMask));
    mzPeak(k) = max(abs(runData(k).Mz(evaluationMask)));
    mzSaturationFraction(k) = mean( ...
        abs(runData(k).Mz(evaluationMask)) >= 0.99*mzMaximum);
end

metrics = table(controller, repmat(evaluationStart, nRuns, 1), ...
    finalYawRate, yawRateRms, yawRatePeak, crossTrackRms, crossTrackPeak, ...
    headingPeakDeg, betaPeakDeg, ayPeak, baseSteeringRmsDeg, ...
    baseSteeringPeakDeg, additionalSteeringRmsDeg, additionalSteeringPeakDeg, ...
    additionalSteeringRatePeakDegS, additionalSteeringSaturationFraction, ...
    totalSteeringRmsDeg, totalSteeringPeakDeg, mzRms, mzPeak, mzSaturationFraction, ...
    'VariableNames', {'Controller', 'EvaluationStart_s', 'FinalYawRate_rad_s', ...
    'YawRateRMSE_rad_s', 'YawRatePeakError_rad_s', 'CrossTrackRMSE_m', ...
    'CrossTrackPeak_m', 'HeadingPeak_deg', 'BetaPeak_deg', 'AyPeak_m_s2', ...
    'BaseSteeringRMSE_deg', 'BaseSteeringPeak_deg', ...
    'AdditionalSteeringRMSE_deg', 'AdditionalSteeringPeak_deg', ...
    'AdditionalSteeringRatePeak_deg_s', 'AdditionalSteeringSaturationFraction', ...
    'TotalSteeringRMSE_deg', 'TotalSteeringPeak_deg', 'MzRMSE_Nm', ...
    'MzPeak_Nm', 'MzSaturationFraction'});
end

function validateRuns(runData, scenario, mzMaximum, afs)

assert(abs(scenario.Vx - 60/3.6) < 1e-12, ...
    "Curved-lane speed must remain 60 km/h.");
assert(abs(scenario.mu - 0.9) < 1e-12, ...
    "Curved-lane friction coefficient must remain 0.9.");
assert(abs(scenario.radius - 100) < 1e-12, ...
    "Curved-lane radius must remain 100 m.");

numericFields = ["X", "Y", "psi", "Vx", "Vy", "r", "beta", "ay", ...
    "deltaBase", "deltaF", "deltaAdd", "deltaAddRate", "Mz", "rRef", ...
    "crossTrackError", "headingError"];

for k = 1:numel(runData)
    for fieldName = numericFields
        values = runData(k).(fieldName);
        assert(all(isfinite(values)), ...
            "%s contains nonfinite values in the %s run.", ...
            fieldName, runData(k).controller);
    end
    assert(max(abs(runData(k).Mz)) ...
        <= mzMaximum + 1e-9, ...
        "%s exceeds the configured yaw-moment limit.", ...
        runData(k).controller);
end

baselineIndex = find([runData.controller] == "Baseline", 1);
afsIndex = find([runData.controller] == "AFS", 1);
tvIndex = find([runData.controller] == "TV", 1);

assert(~isempty(baselineIndex), "Baseline run is missing.");
assert(~isempty(afsIndex), "AFS run is missing.");
assert(~isempty(tvIndex), "TV run is missing.");

%% Baseline invariants
assert(max(abs(runData(baselineIndex).Mz)) <= 1e-12, ...
    "Baseline must command zero yaw moment.");
assert(max(abs(runData(baselineIndex).deltaAdd)) <= 1e-12, ...
    "Baseline must command zero additional steering.");

%% AFS invariants
assert(max(abs(runData(afsIndex).Mz)) <= 1e-12, ...
    "AFS must command zero yaw moment.");
assert(max(abs(runData(afsIndex).deltaAdd)) > 1e-8, ...
    "AFS run did not produce additional steering.");
assert(max(abs(runData(afsIndex).deltaAdd)) ...
    <= afs.deltaMax + 1e-8, ...
    "AFS exceeded its steering-correction limit.");
assert(max(abs(runData(afsIndex).deltaF)) ...
    <= afs.totalDeltaMax + 1e-8, ...
    "AFS exceeded the total-steering limit.");

%% TV invariants
assert(max(abs(runData(tvIndex).deltaAdd)) <= 1e-12, ...
    "TV must command zero additional steering.");
assert(max(abs(runData(tvIndex).Mz)) > 0, ...
    "TV run did not produce a yaw-moment command.");
end

function createTrackingFigure(runData, evaluationStart, outputDirectory)
colors = lines(numel(runData));
figureHandle = figure("Visible", "off", "Color", "w", ...
    "Position", [100 100 1200 780]);
layout = tiledlayout(2, 2, "TileSpacing", "compact", ...
    "Padding", "compact");
title(layout, "Week 1 curved-lane tracking comparison", "Color", "k");

nexttile;
plot(runData(1).XRef, runData(1).YRef, "k--", "LineWidth", 1.5, ...
    "DisplayName", "Reference");
hold on;
for k = 1:numel(runData)
    plot(runData(k).X, runData(k).Y, "LineWidth", 1.4, ...
        "Color", colors(k, :), "DisplayName", runData(k).controller);
end
axis equal;
grid on;
xlabel("X [m]");
ylabel("Y [m]");
title("Trajectory");
legend("Location", "best");

nexttile;
plot(runData(1).time, runData(1).rRef, "k--", "LineWidth", 1.5, ...
    "DisplayName", "r_{ref}");
hold on;
for k = 1:numel(runData)
    plot(runData(k).time, runData(k).r, "LineWidth", 1.4, ...
        "Color", colors(k, :), "DisplayName", runData(k).controller);
end
xline(evaluationStart, ":", "Evaluation start", ...
    "HandleVisibility", "off");
grid on;
xlabel("Time [s]");
ylabel("Yaw rate [rad/s]");
title("Yaw-rate tracking");
legend("Location", "best");

nexttile;
hold on;
for k = 1:numel(runData)
    plot(runData(k).time, runData(k).crossTrackError, ...
        "LineWidth", 1.4, "Color", colors(k, :), ...
        "DisplayName", runData(k).controller);
end
xline(evaluationStart, ":", "Evaluation start", ...
    "HandleVisibility", "off");
grid on;
xlabel("Time [s]");
ylabel("Cross-track error [m]");
title("Path error");
legend("Location", "best");

nexttile;
hold on;
for k = 1:numel(runData)
    plot(runData(k).time, rad2deg(runData(k).beta), ...
        "LineWidth", 1.4, "Color", colors(k, :), ...
        "DisplayName", runData(k).controller);
end
xline(evaluationStart, ":", "Evaluation start", ...
    "HandleVisibility", "off");
grid on;
xlabel("Time [s]");
ylabel("Sideslip angle [deg]");
title("Sideslip response");
legend("Location", "best");

applyLightFigureStyle(figureHandle);
exportgraphics(figureHandle, ...
    fullfile(outputDirectory, "tracking_comparison.png"), ...
    "Resolution", 180);
close(figureHandle);
end

function createEffortFigure(runData, evaluationStart, outputDirectory)

colors = lines(numel(runData));
figureHandle = figure("Visible", "off", "Color", "w", ...
    "Position", [100 50 1150 900]);
layout = tiledlayout(4, 1, "TileSpacing", "compact", ...
    "Padding", "compact");
title(layout, "Curved-lane vehicle response and control effort", "Color", "k");

%% Lateral acceleration

nexttile;
hold on;
for k = 1:numel(runData)
    plot(runData(k).time, runData(k).ay, "LineWidth", 1.4, ...
        "Color", colors(k,:), "DisplayName", runData(k).controller);
end
xline(evaluationStart, ":", "Evaluation start", "HandleVisibility", "off");
grid on;
xlabel("Time [s]");
ylabel("a_y [m/s^2]");
title("Lateral acceleration");
legend("Location", "best");

%% Total front steering
nexttile;
hold on;
for k = 1:numel(runData)
    plot(runData(k).time, rad2deg(runData(k).deltaF), "LineWidth", 1.4, ...
        "Color", colors(k,:), "DisplayName", runData(k).controller);
end
xline(evaluationStart, ":", "Evaluation start", "HandleVisibility", "off");
grid on;
xlabel("Time [s]");
ylabel("\delta_f [deg]");
title("Total front steering");
legend("Location", "best");

%% Additional AFS steering
nexttile;
hold on;
for k = 1:numel(runData)
    plot(runData(k).time, rad2deg(runData(k).deltaAdd), "LineWidth", 1.4, ...
        "Color", colors(k,:), "DisplayName", runData(k).controller);
end
xline(evaluationStart, ":", "Evaluation start", "HandleVisibility", "off");
grid on;
xlabel("Time [s]");
ylabel("\delta_{add} [deg]");
title("Active-front-steering effort");
legend("Location", "best");

%% Direct yaw moment
nexttile;
hold on;
for k = 1:numel(runData)
    plot(runData(k).time, runData(k).Mz/1000, "LineWidth", 1.4, ...
        "Color", colors(k,:), "DisplayName", runData(k).controller);
end
xline(evaluationStart, ":", "Evaluation start", "HandleVisibility", "off");
grid on;
xlabel("Time [s]");
ylabel("M_z [kN m]");
title("Torque-vectoring effort");
legend("Location", "best");

applyLightFigureStyle(figureHandle);
exportgraphics(figureHandle, ...
    fullfile(outputDirectory, "control_effort.png"), "Resolution", 180);
close(figureHandle);
end

function applyLightFigureStyle(figureHandle)
% Keep report figures readable regardless of the MATLAB desktop theme.
axesHandles = findall(figureHandle, "Type", "axes");
set(axesHandles, "Color", "w", "XColor", "k", "YColor", "k", ...
    "GridColor", [0.75 0.75 0.75]);
set(findall(figureHandle, "Type", "text"), "Color", "k");
legendHandles = findall(figureHandle, "Type", "legend");
set(legendHandles, "Color", "w", "TextColor", "k", ...
    "EdgeColor", [0.25 0.25 0.25]);
end

function signal = requireSignal(sdiRun, requestedName)
signalIds = sdiRun.getAllSignalIDs;
signalNames = strings(size(signalIds));
for k = 1:numel(signalIds)
    signalNames(k) = string(Simulink.sdi.getSignal(signalIds(k)).Name);
end

match = find(signalNames == requestedName);
if numel(match) ~= 1
    error("runCurvedLaneComparison:SignalLookup", ...
        "Expected one signal named '%s'; found %d.", ...
        requestedName, numel(match));
end
signal = Simulink.sdi.getSignal(signalIds(match)).Values;
end

function values = sampleSignal(signal, queryTime)
signalTime = double(signal.Time(:));
signalData = double(signal.Data(:));
if isscalar(signalTime)
    values = repmat(signalData, size(queryTime));
else
    values = interp1(signalTime, signalData, queryTime, "linear", "extrap");
end
end

function output = firstOrderResponse(time, input, timeConstant)
output = zeros(size(input));
for k = 2:numel(time)
    timeStep = time(k) - time(k - 1);
    decay = exp(-timeStep/timeConstant);
    output(k) = decay*output(k - 1) + (1 - decay)*input(k - 1);
end
end

function derivative = numericalDerivative(time, values)
%numericalDerivative Calculate a derivative for nonuniform time samples.

time = double(time(:));
values = double(values(:));

derivative = zeros(size(values));

if numel(values) < 2
    return
end

dt = diff(time);
dv = diff(values);

validStep = dt > eps;

rateValues = zeros(size(dv));
rateValues(validStep) = dv(validStep)./dt(validStep);

derivative(2:end) = rateValues;
derivative(1) = derivative(2);
end

function value = rootMeanSquare(values)
value = sqrt(mean(values.^2));
end

function data = emptyRunData()
data = struct( ...
    "controller", "", ...
    "time", [], ...
    "X", [], "Y", [], "psi", [], "Vx", [], "Vy", [], ...
    "r", [], "beta", [], "ay", [], ...
    "XRef", [], "YRef", [], "psiRef", [], "kappaRef", [], ...
    "mu", [], "deltaBase", [], "deltaF", [], "deltaAdd", [], ...
    "deltaAddRate", [], "Mz", [], "rTarget", [], "rRef", [], ...
    "crossTrackError", [], "headingError", [], ...
    "yawRateError", [], "positionError", [], ...
    "scenarioSpeed", [], "scenarioRadius", []);
end

function restoreControllerMode(hadControllerMode, originalControllerMode)
if hadControllerMode
    assignin("base", "controllerMode", originalControllerMode);
else
    evalin("base", "clear controllerMode");
end
end
