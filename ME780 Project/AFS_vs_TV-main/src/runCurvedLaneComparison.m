function results = runCurvedLaneComparison()
%runCurvedLaneComparison Compare Baseline and TV on the curved-lane test.
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
outputDirectory = fullfile(projectRoot, "results", "week1_curved_lane");
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
controllerModes = [evalin("base", "CTRL_BASELINE"), ...
                   evalin("base", "CTRL_TV")];
controllerNames = ["Baseline", "TV"];
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
    runData(k) = extractRunData(sdiRun, controllerNames(k), scenario, tv);
end

evaluationStart = scenario.startTime + 1.0;
metrics = calculateMetrics(runData, evaluationStart, tv.MzMax);

% Acceptance invariants come from the model interface and scenario, rather
% than arbitrary performance thresholds.
validateRuns(runData, scenario, tv.MzMax);

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

fprintf("\nCurved-lane Baseline versus TV comparison\n");
disp(metrics);
fprintf("Results written to:\n  %s\n", outputDirectory);

% Retain cleanup objects until all output has been written.
clear modeCleanup directoryCleanup
end

function data = extractRunData(sdiRun, controllerName, scenario, tv)
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

speedForLimit = max(abs(data.Vx), 0.1);
yawRateLimit = data.mu .* 9.81 ./ speedForLimit;
data.rTarget = min(max(data.Vx .* data.kappaRef, -yawRateLimit), ...
                   yawRateLimit);
data.rRef = firstOrderResponse(time, data.rTarget, tv.tauRef);

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

function metrics = calculateMetrics(runData, evaluationStart, mzMaximum)
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
steeringRmsDeg = zeros(nRuns, 1);
steeringPeakDeg = zeros(nRuns, 1);
mzRms = zeros(nRuns, 1);
mzPeak = zeros(nRuns, 1);
mzSaturationFraction = zeros(nRuns, 1);

for k = 1:nRuns
    evaluationMask = runData(k).time >= evaluationStart;
    controller(k) = runData(k).controller;
    finalYawRate(k) = runData(k).r(end);
    yawRateRms(k) = rootMeanSquare( ...
        runData(k).yawRateError(evaluationMask));
    yawRatePeak(k) = max(abs(runData(k).yawRateError(evaluationMask)));
    crossTrackRms(k) = rootMeanSquare( ...
        runData(k).crossTrackError(evaluationMask));
    crossTrackPeak(k) = max(abs( ...
        runData(k).crossTrackError(evaluationMask)));
    headingPeakDeg(k) = rad2deg(max(abs( ...
        runData(k).headingError(evaluationMask))));
    betaPeakDeg(k) = rad2deg(max(abs(runData(k).beta(evaluationMask))));
    ayPeak(k) = max(abs(runData(k).ay(evaluationMask)));
    steeringRmsDeg(k) = rad2deg(rootMeanSquare( ...
        runData(k).deltaF(evaluationMask)));
    steeringPeakDeg(k) = rad2deg(max(abs( ...
        runData(k).deltaF(evaluationMask))));
    mzRms(k) = rootMeanSquare(runData(k).Mz(evaluationMask));
    mzPeak(k) = max(abs(runData(k).Mz(evaluationMask)));
    mzSaturationFraction(k) = mean( ...
        abs(runData(k).Mz(evaluationMask)) >= 0.99*mzMaximum);
end

metrics = table(controller, repmat(evaluationStart, nRuns, 1), ...
    finalYawRate, yawRateRms, yawRatePeak, crossTrackRms, ...
    crossTrackPeak, headingPeakDeg, betaPeakDeg, ayPeak, ...
    steeringRmsDeg, steeringPeakDeg, mzRms, mzPeak, ...
    mzSaturationFraction, ...
    'VariableNames', {'Controller', 'EvaluationStart_s', ...
    'FinalYawRate_rad_s', 'YawRateRMSE_rad_s', ...
    'YawRatePeakError_rad_s', 'CrossTrackRMSE_m', ...
    'CrossTrackPeak_m', 'HeadingPeak_deg', 'BetaPeak_deg', ...
    'AyPeak_m_s2', 'SteeringRMSE_deg', 'SteeringPeak_deg', ...
    'MzRMSE_Nm', 'MzPeak_Nm', 'MzSaturationFraction'});
end

function validateRuns(runData, scenario, mzMaximum)
assert(abs(scenario.Vx - 60/3.6) < 1e-12, ...
    "Curved-lane speed must remain 60 km/h for the Week 1 comparison.");
assert(abs(scenario.mu - 0.9) < 1e-12, ...
    "Curved-lane friction coefficient must remain 0.9.");
assert(abs(scenario.radius - 100) < 1e-12, ...
    "Curved-lane radius must remain 100 m.");

numericFields = ["X", "Y", "psi", "Vx", "Vy", "r", "beta", ...
    "ay", "deltaF", "Mz", "rRef", "crossTrackError", ...
    "headingError"];
for k = 1:numel(runData)
    for fieldName = numericFields
        values = runData(k).(fieldName);
        assert(all(isfinite(values)), ...
            "%s contains nonfinite values in the %s run.", ...
            fieldName, runData(k).controller);
    end
    assert(max(abs(runData(k).Mz)) <= mzMaximum + 1e-9, ...
        "%s exceeds the configured yaw-moment limit.", ...
        runData(k).controller);
end

baselineIndex = find([runData.controller] == "Baseline", 1);
tvIndex = find([runData.controller] == "TV", 1);
assert(max(abs(runData(baselineIndex).Mz)) <= 1e-12, ...
    "Baseline must command zero yaw moment.");
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
    "Position", [100 100 1100 720]);
layout = tiledlayout(3, 1, "TileSpacing", "compact", ...
    "Padding", "compact");
title(layout, "Week 1 vehicle response and control effort", "Color", "k");

nexttile;
hold on;
for k = 1:numel(runData)
    plot(runData(k).time, runData(k).ay, "LineWidth", 1.4, ...
        "Color", colors(k, :), "DisplayName", runData(k).controller);
end
xline(evaluationStart, ":", "Evaluation start", ...
    "HandleVisibility", "off");
grid on;
xlabel("Time [s]");
ylabel("a_y [m/s^2]");
title("Lateral acceleration");
legend("Location", "best");

nexttile;
hold on;
for k = 1:numel(runData)
    plot(runData(k).time, rad2deg(runData(k).deltaF), ...
        "LineWidth", 1.4, "Color", colors(k, :), ...
        "DisplayName", runData(k).controller);
end
xline(evaluationStart, ":", "Evaluation start", ...
    "HandleVisibility", "off");
grid on;
xlabel("Time [s]");
ylabel("Front steer [deg]");
title("Steering effort");
legend("Location", "best");

nexttile;
hold on;
for k = 1:numel(runData)
    plot(runData(k).time, runData(k).Mz/1000, "LineWidth", 1.4, ...
        "Color", colors(k, :), "DisplayName", runData(k).controller);
end
xline(evaluationStart, ":", "Evaluation start", ...
    "HandleVisibility", "off");
grid on;
xlabel("Time [s]");
ylabel("M_z [kN m]");
title("Torque-vectoring effort");
legend("Location", "best");

applyLightFigureStyle(figureHandle);
exportgraphics(figureHandle, ...
    fullfile(outputDirectory, "control_effort.png"), ...
    "Resolution", 180);
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
    "Mz", [], "rTarget", [], "rRef", [], ...
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
