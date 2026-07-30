function results = runDLCComparison()
%runDLCComparison Compare Baseline, AFS, and TV on a dry-road DLC.
%
% Usage from the project root:
%   setupProject
%   resultsDLC = runDLCComparison();

projectRoot = fileparts(fileparts(mfilename("fullpath")));
outputDirectory = fullfile(projectRoot, "results", "dlc_70kph_mu09");
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

%% Initialize project and select the DLC scenario

evalin("base", "setupProject");

scenario = DLCParams();
assignin("base", "scenario", scenario);

tv = evalin("base", "tv");
afs = evalin("base", "afs");

controllerModes = [ ...
    evalin("base", "CTRL_BASELINE"), ...
    evalin("base", "CTRL_AFS"), ...
    evalin("base", "CTRL_TV")];

controllerNames = ["Baseline", "AFS", "TV"];
modelName = "experimentHarnessDLC";
stopTime = scenario.stopTime;

clear solveAFSMPCOnline

%% Run all controller cases

runData = repmat(emptyRunData(), 1, numel(controllerModes));

for k = 1:numel(controllerModes)
    fprintf("Running %s DLC case...\n", controllerNames(k));

    previousRunIds = Simulink.sdi.getAllRunIDs;

    simInput = Simulink.SimulationInput(modelName);
    simInput = simInput.setVariable("scenario", scenario);
    simInput = simInput.setVariable("controllerMode", controllerModes(k));
    simInput = simInput.setModelParameter( ...
        "StopTime", num2str(stopTime, "%.15g"));

    simulationOutput = sim(simInput); %#ok<NASGU>

    currentRunIds = Simulink.sdi.getAllRunIDs;
    newRunIds = setdiff(currentRunIds, previousRunIds, "stable");

    if isempty(newRunIds)
        error("runDLCComparison:MissingSDIRun", ...
            "Simulation Data Inspector did not record the %s run.", ...
            controllerNames(k));
    end

    sdiRun = Simulink.sdi.getRun(newRunIds(end));
    runData(k) = extractRunData( ...
        sdiRun, controllerNames(k), scenario, afs.Ts);
end

%% Metrics and validation

evaluationStart = scenario.evaluationStart;
evaluationEnd = scenario.evaluationEnd;

metrics = calculateMetrics( ...
    runData, evaluationStart, evaluationEnd, ...
    tv.MzMax, afs.deltaMax);

validateRuns(runData, scenario, tv.MzMax, afs);

%% Save results

writetable(metrics, fullfile(outputDirectory, "metrics.csv"));

createTrackingFigure( ...
    runData, evaluationStart, evaluationEnd, outputDirectory);

createEffortFigure( ...
    runData, evaluationStart, evaluationEnd, outputDirectory);

results = struct;
results.generatedAt = string(datetime("now", "TimeZone", "local"));
results.scenario = scenario;
results.evaluationStart = evaluationStart;
results.evaluationEnd = evaluationEnd;
results.metrics = metrics;
results.runs = runData;
results.outputDirectory = string(outputDirectory);

save(fullfile(outputDirectory, "comparison_results.mat"), "results");

fprintf("\nDouble-lane-change comparison: 70 km/h, mu = 0.9\n");
disp(metrics);
fprintf("Results written to:\n  %s\n", outputDirectory);

clear modeCleanup directoryCleanup
end


function data = extractRunData(sdiRun, controllerName, scenario, afsSampleTime)
%extractRunData Read one controller run from SDI.

xSignal = requireSignal(sdiRun, "Plant:1.X");
time = double(xSignal.Time(:));

data = emptyRunData();
data.controller = controllerName;
data.time = time;

% Plant states
data.X = sampleSignal(xSignal, time);
data.Y = sampleSignal(requireSignal(sdiRun, "Plant:1.Y"), time);
data.psi = sampleSignal(requireSignal(sdiRun, "Plant:1.psi"), time);
data.Vx = sampleSignal(requireSignal(sdiRun, "Plant:1.Vx"), time);
data.Vy = sampleSignal(requireSignal(sdiRun, "Plant:1.Vy"), time);
data.r = sampleSignal(requireSignal(sdiRun, "Plant:1.r"), time);
data.beta = sampleSignal(requireSignal(sdiRun, "Plant:1.beta"), time);
data.ay = sampleSignal(requireSignal(sdiRun, "Plant:1.ay"), time);

% Reference bus
data.XRef = sampleSignal( ...
    requireSignal(sdiRun, "Ref Bus:1.X_ref"), time);
data.YRef = sampleSignal( ...
    requireSignal(sdiRun, "Ref Bus:1.Y_ref"), time);
data.psiRef = sampleSignal( ...
    requireSignal(sdiRun, "Ref Bus:1.psi_ref"), time);
data.kappaRef = sampleSignal( ...
    requireSignal(sdiRun, "Ref Bus:1.kappa_ref"), time);
data.mu = sampleSignal( ...
    requireSignal(sdiRun, "Ref Bus:1.mu"), time);

% Controller commands
data.deltaBase = sampleSignal( ...
    requireSignal(sdiRun, "BaseSteering:1"), time);
data.deltaF = sampleSignal( ...
    requireSignal(sdiRun, "Controller:1"), time);
data.Mz = sampleSignal( ...
    requireSignal(sdiRun, "Controller:2"), time);
data.deltaAdd = data.deltaF - data.deltaBase;
data.deltaAddRate = sampledCommandDerivative( ...
    time, data.deltaAdd, afsSampleTime);

% Common path-based yaw-rate reference used only for metrics.
speedForLimit = max(abs(data.Vx), 0.1);
yawRateLimit = data.mu .* 9.81 ./ speedForLimit;
data.rTarget = data.Vx .* data.kappaRef;
data.rRef = min(max(data.rTarget, -yawRateLimit), yawRateLimit);

% Tracking errors
data.crossTrackError = ...
    -sin(data.psiRef) .* (data.X - data.XRef) ...
    + cos(data.psiRef) .* (data.Y - data.YRef);

data.headingError = atan2( ...
    sin(data.psiRef - data.psi), ...
    cos(data.psiRef - data.psi));

data.yawRateError = data.rRef - data.r;
data.positionError = hypot(data.X - data.XRef, data.Y - data.YRef);

% Scenario metadata
data.scenarioSpeed = scenario.Vx;
data.scenarioLaneWidth = scenario.laneWidth;
data.scenarioMu = scenario.mu;
end


function metrics = calculateMetrics( ...
    runData, evaluationStart, evaluationEnd, ...
    mzMaximum, afsMaximum)
%calculateMetrics Calculate tracking, stability, and effort metrics.

nRuns = numel(runData);
controller = strings(nRuns, 1);

finalCrossTrack = zeros(nRuns, 1);
finalHeadingDeg = zeros(nRuns, 1);
finalYawRate = zeros(nRuns, 1);
yawRateRms = zeros(nRuns, 1);
yawRatePeak = zeros(nRuns, 1);
crossTrackRms = zeros(nRuns, 1);
crossTrackPeak = zeros(nRuns, 1);
headingRmsDeg = zeros(nRuns, 1);
headingPeakDeg = zeros(nRuns, 1);
betaRmsDeg = zeros(nRuns, 1);
betaPeakDeg = zeros(nRuns, 1);
ayRms = zeros(nRuns, 1);
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
    evaluationMask = ...
        runData(k).time >= evaluationStart ...
        & runData(k).time <= evaluationEnd;

    if ~any(evaluationMask)
        error("runDLCComparison:EmptyEvaluationWindow", ...
            "No samples fall inside the evaluation window for %s.", ...
            runData(k).controller);
    end

    controller(k) = runData(k).controller;
    finalCrossTrack(k) = runData(k).crossTrackError(end);
    finalHeadingDeg(k) = rad2deg(runData(k).headingError(end));
    finalYawRate(k) = runData(k).r(end);

    yawRateRms(k) = rootMeanSquare( ...
        runData(k).yawRateError(evaluationMask));
    yawRatePeak(k) = max(abs( ...
        runData(k).yawRateError(evaluationMask)));

    crossTrackRms(k) = rootMeanSquare( ...
        runData(k).crossTrackError(evaluationMask));
    crossTrackPeak(k) = max(abs( ...
        runData(k).crossTrackError(evaluationMask)));

    headingRmsDeg(k) = rad2deg(rootMeanSquare( ...
        runData(k).headingError(evaluationMask)));
    headingPeakDeg(k) = rad2deg(max(abs( ...
        runData(k).headingError(evaluationMask))));

    betaRmsDeg(k) = rad2deg(rootMeanSquare( ...
        runData(k).beta(evaluationMask)));
    betaPeakDeg(k) = rad2deg(max(abs( ...
        runData(k).beta(evaluationMask))));

    ayRms(k) = rootMeanSquare(runData(k).ay(evaluationMask));
    ayPeak(k) = max(abs(runData(k).ay(evaluationMask)));

    baseSteeringRmsDeg(k) = rad2deg(rootMeanSquare( ...
        runData(k).deltaBase(evaluationMask)));
    baseSteeringPeakDeg(k) = rad2deg(max(abs( ...
        runData(k).deltaBase(evaluationMask))));

    additionalSteeringRmsDeg(k) = rad2deg(rootMeanSquare( ...
        runData(k).deltaAdd(evaluationMask)));
    additionalSteeringPeakDeg(k) = rad2deg(max(abs( ...
        runData(k).deltaAdd(evaluationMask))));
    additionalSteeringRatePeakDegS(k) = rad2deg(max(abs( ...
        runData(k).deltaAddRate(evaluationMask))));
    additionalSteeringSaturationFraction(k) = mean( ...
        abs(runData(k).deltaAdd(evaluationMask)) ...
        >= 0.99*afsMaximum);

    totalSteeringRmsDeg(k) = rad2deg(rootMeanSquare( ...
        runData(k).deltaF(evaluationMask)));
    totalSteeringPeakDeg(k) = rad2deg(max(abs( ...
        runData(k).deltaF(evaluationMask))));

    mzRms(k) = rootMeanSquare(runData(k).Mz(evaluationMask));
    mzPeak(k) = max(abs(runData(k).Mz(evaluationMask)));
    mzSaturationFraction(k) = mean( ...
        abs(runData(k).Mz(evaluationMask)) >= 0.99*mzMaximum);
end

metrics = table( ...
    controller, ...
    repmat(evaluationStart, nRuns, 1), ...
    repmat(evaluationEnd, nRuns, 1), ...
    finalCrossTrack, finalHeadingDeg, finalYawRate, ...
    yawRateRms, yawRatePeak, ...
    crossTrackRms, crossTrackPeak, ...
    headingRmsDeg, headingPeakDeg, ...
    betaRmsDeg, betaPeakDeg, ...
    ayRms, ayPeak, ...
    baseSteeringRmsDeg, baseSteeringPeakDeg, ...
    additionalSteeringRmsDeg, additionalSteeringPeakDeg, ...
    additionalSteeringRatePeakDegS, ...
    additionalSteeringSaturationFraction, ...
    totalSteeringRmsDeg, totalSteeringPeakDeg, ...
    mzRms, mzPeak, mzSaturationFraction, ...
    'VariableNames', { ...
    'Controller', 'EvaluationStart_s', 'EvaluationEnd_s', ...
    'FinalCrossTrack_m', 'FinalHeading_deg', ...
    'FinalYawRate_rad_s', ...
    'YawRateRMSE_rad_s', 'YawRatePeakError_rad_s', ...
    'CrossTrackRMSE_m', 'CrossTrackPeak_m', ...
    'HeadingRMSE_deg', 'HeadingPeak_deg', ...
    'BetaRMSE_deg', 'BetaPeak_deg', ...
    'AyRMSE_m_s2', 'AyPeak_m_s2', ...
    'BaseSteeringRMSE_deg', 'BaseSteeringPeak_deg', ...
    'AdditionalSteeringRMSE_deg', ...
    'AdditionalSteeringPeak_deg', ...
    'AdditionalSteeringRatePeak_deg_s', ...
    'AdditionalSteeringSaturationFraction', ...
    'TotalSteeringRMSE_deg', 'TotalSteeringPeak_deg', ...
    'MzRMSE_Nm', 'MzPeak_Nm', 'MzSaturationFraction'});
end


function validateRuns(runData, scenario, mzMaximum, afs)
%validateRuns Check scenario conditions and controller invariants.

assert(abs(scenario.Vx - 70/3.6) < 1e-12, ...
    "DLC speed must remain 70 km/h.");
assert(abs(scenario.mu - 0.9) < 1e-12, ...
    "DLC friction coefficient must remain 0.9.");
assert(abs(scenario.laneWidth - 3.5) < 1e-12, ...
    "DLC lane displacement must remain 3.5 m.");
assert(scenario.transitionLength > 0, ...
    "DLC transition length must be positive.");
assert(scenario.holdLength >= 0, ...
    "DLC hold length cannot be negative.");
assert(scenario.xEnd > scenario.xStart, ...
    "DLC end position must be after its start position.");
assert(scenario.evaluationEnd > scenario.evaluationStart, ...
    "DLC evaluation end must be after its start.");

numericFields = [ ...
    "X", "Y", "psi", "Vx", "Vy", "r", "beta", "ay", ...
    "XRef", "YRef", "psiRef", "kappaRef", "mu", ...
    "deltaBase", "deltaF", "deltaAdd", "deltaAddRate", ...
    "Mz", "rRef", "crossTrackError", "headingError", ...
    "yawRateError", "positionError"];

for k = 1:numel(runData)
    for fieldName = numericFields
        values = runData(k).(fieldName);
        assert(all(isfinite(values)), ...
            "%s contains nonfinite values in the %s run.", ...
            fieldName, runData(k).controller);
    end

    assert(max(abs(runData(k).Mz)) <= mzMaximum + 1e-8, ...
        "%s exceeds the configured yaw-moment limit.", ...
        runData(k).controller);

    assert(max(abs(runData(k).deltaF)) ...
        <= afs.totalDeltaMax + 1e-8, ...
        "%s exceeds the configured total-steering limit.", ...
        runData(k).controller);
end

referenceRun = runData(1);
assert(abs(referenceRun.YRef(1)) < 1e-6, ...
    "DLC reference must begin in the original lane.");
assert(abs(max(referenceRun.YRef) - scenario.laneWidth) < 1e-3, ...
    "DLC reference did not reach the adjacent lane.");
assert(abs(referenceRun.YRef(end)) < 1e-3, ...
    "DLC reference must finish in the original lane.");

baselineIndex = find([runData.controller] == "Baseline", 1);
afsIndex = find([runData.controller] == "AFS", 1);
tvIndex = find([runData.controller] == "TV", 1);

assert(~isempty(baselineIndex), "Baseline run is missing.");
assert(~isempty(afsIndex), "AFS run is missing.");
assert(~isempty(tvIndex), "TV run is missing.");

assert(max(abs(runData(baselineIndex).Mz)) <= 1e-12, ...
    "Baseline must command zero yaw moment.");
assert(max(abs(runData(baselineIndex).deltaAdd)) <= 1e-12, ...
    "Baseline must command zero additional steering.");

assert(max(abs(runData(afsIndex).Mz)) <= 1e-12, ...
    "AFS must command zero yaw moment.");
assert(max(abs(runData(afsIndex).deltaAdd)) > 1e-10, ...
    "AFS did not produce an additional steering command.");
assert(max(abs(runData(afsIndex).deltaAdd)) ...
    <= afs.deltaMax + 1e-8, ...
    "AFS exceeded its steering-correction limit.");

assert(max(abs(runData(tvIndex).deltaAdd)) <= 1e-12, ...
    "TV must command zero additional steering.");
assert(max(abs(runData(tvIndex).Mz)) > 0, ...
    "TV did not produce a yaw-moment command.");
end


function createTrackingFigure( ...
    runData, evaluationStart, evaluationEnd, outputDirectory)
%createTrackingFigure Create DLC path and tracking plots.

colors = lines(numel(runData));
figureHandle = figure("Visible", "off", "Color", "w", ...
    "Position", [100 50 1250 900]);
layout = tiledlayout(3, 2, "TileSpacing", "compact", ...
    "Padding", "compact");
title(layout, "Double lane change: 70 km/h, \mu = 0.9", ...
    "Color", "k");

nexttile;
plot(runData(1).XRef, runData(1).YRef, "k--", ...
    "LineWidth", 1.5, "DisplayName", "Reference");
hold on;
for k = 1:numel(runData)
    plot(runData(k).X, runData(k).Y, "LineWidth", 1.4, ...
        "Color", colors(k,:), "DisplayName", runData(k).controller);
end
axis equal; grid on;
xlabel("X [m]"); ylabel("Y [m]");
title("Vehicle trajectory");
legend("Location", "best");

nexttile;
plot(runData(1).time, runData(1).YRef, "k--", ...
    "LineWidth", 1.5, "DisplayName", "Y_{ref}");
hold on;
for k = 1:numel(runData)
    plot(runData(k).time, runData(k).Y, "LineWidth", 1.4, ...
        "Color", colors(k,:), "DisplayName", runData(k).controller);
end
addEvaluationLines(evaluationStart, evaluationEnd);
grid on; xlabel("Time [s]"); ylabel("Y [m]");
title("Lateral-position tracking");
legend("Location", "best");

nexttile;
plot(runData(1).time, runData(1).rRef, "k--", ...
    "LineWidth", 1.5, "DisplayName", "r_{path,ref}");
hold on;
for k = 1:numel(runData)
    plot(runData(k).time, runData(k).r, "LineWidth", 1.4, ...
        "Color", colors(k,:), "DisplayName", runData(k).controller);
end
addEvaluationLines(evaluationStart, evaluationEnd);
grid on; xlabel("Time [s]"); ylabel("Yaw rate [rad/s]");
title("Yaw-rate tracking");
legend("Location", "best");

nexttile;
hold on;
for k = 1:numel(runData)
    plot(runData(k).time, runData(k).crossTrackError, ...
        "LineWidth", 1.4, "Color", colors(k,:), ...
        "DisplayName", runData(k).controller);
end
addEvaluationLines(evaluationStart, evaluationEnd);
grid on; xlabel("Time [s]"); ylabel("Cross-track error [m]");
title("Cross-track error");
legend("Location", "best");

nexttile;
hold on;
for k = 1:numel(runData)
    plot(runData(k).time, rad2deg(runData(k).headingError), ...
        "LineWidth", 1.4, "Color", colors(k,:), ...
        "DisplayName", runData(k).controller);
end
addEvaluationLines(evaluationStart, evaluationEnd);
grid on; xlabel("Time [s]"); ylabel("Heading error [deg]");
title("Heading error");
legend("Location", "best");

nexttile;
hold on;
for k = 1:numel(runData)
    plot(runData(k).time, rad2deg(runData(k).beta), ...
        "LineWidth", 1.4, "Color", colors(k,:), ...
        "DisplayName", runData(k).controller);
end
addEvaluationLines(evaluationStart, evaluationEnd);
grid on; xlabel("Time [s]"); ylabel("Sideslip angle [deg]");
title("Sideslip response");
legend("Location", "best");

applyLightFigureStyle(figureHandle);
exportgraphics(figureHandle, ...
    fullfile(outputDirectory, "tracking_comparison.png"), ...
    "Resolution", 180);
close(figureHandle);
end


function createEffortFigure( ...
    runData, evaluationStart, evaluationEnd, outputDirectory)
%createEffortFigure Create response and effort plots.

colors = lines(numel(runData));
figureHandle = figure("Visible", "off", "Color", "w", ...
    "Position", [100 50 1150 900]);
layout = tiledlayout(4, 1, "TileSpacing", "compact", ...
    "Padding", "compact");
title(layout, "DLC vehicle response and control effort", "Color", "k");

nexttile; hold on;
for k = 1:numel(runData)
    plot(runData(k).time, runData(k).ay, "LineWidth", 1.4, ...
        "Color", colors(k,:), "DisplayName", runData(k).controller);
end
addEvaluationLines(evaluationStart, evaluationEnd);
grid on; xlabel("Time [s]"); ylabel("a_y [m/s^2]");
title("Lateral acceleration");
legend("Location", "best");

nexttile; hold on;
for k = 1:numel(runData)
    plot(runData(k).time, rad2deg(runData(k).deltaF), ...
        "LineWidth", 1.4, "Color", colors(k,:), ...
        "DisplayName", runData(k).controller);
end
addEvaluationLines(evaluationStart, evaluationEnd);
grid on; xlabel("Time [s]"); ylabel("\delta_f [deg]");
title("Total front steering");
legend("Location", "best");

nexttile; hold on;
for k = 1:numel(runData)
    plot(runData(k).time, rad2deg(runData(k).deltaAdd), ...
        "LineWidth", 1.4, "Color", colors(k,:), ...
        "DisplayName", runData(k).controller);
end
addEvaluationLines(evaluationStart, evaluationEnd);
grid on; xlabel("Time [s]"); ylabel("\delta_{add} [deg]");
title("Active-front-steering effort");
legend("Location", "best");

nexttile; hold on;
for k = 1:numel(runData)
    plot(runData(k).time, runData(k).Mz/1000, ...
        "LineWidth", 1.4, "Color", colors(k,:), ...
        "DisplayName", runData(k).controller);
end
addEvaluationLines(evaluationStart, evaluationEnd);
grid on; xlabel("Time [s]"); ylabel("M_z [kN m]");
title("Torque-vectoring effort");
legend("Location", "best");

applyLightFigureStyle(figureHandle);
exportgraphics(figureHandle, ...
    fullfile(outputDirectory, "control_effort.png"), ...
    "Resolution", 180);
close(figureHandle);
end


function addEvaluationLines(evaluationStart, evaluationEnd)
xline(evaluationStart, ":", "Evaluation start", ...
    "HandleVisibility", "off");
xline(evaluationEnd, ":", "Evaluation end", ...
    "HandleVisibility", "off");
end


function applyLightFigureStyle(figureHandle)
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
    error("runDLCComparison:SignalLookup", ...
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
    values = interp1(signalTime, signalData, queryTime, ...
        "linear", "extrap");
end
end


function derivative = numericalDerivative(time, values)
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
    "mu", [], "deltaBase", [], "deltaF", [], ...
    "deltaAdd", [], "deltaAddRate", [], "Mz", [], ...
    "rTarget", [], "rRef", [], ...
    "crossTrackError", [], "headingError", [], ...
    "yawRateError", [], "positionError", [], ...
    "scenarioSpeed", [], "scenarioLaneWidth", [], ...
    "scenarioMu", []);
end


function restoreControllerMode(hadControllerMode, originalControllerMode)
if hadControllerMode
    assignin("base", "controllerMode", originalControllerMode);
else
    evalin("base", "clear controllerMode");
end
end

function rateAtTime = sampledCommandDerivative( ...
    time, command, sampleTime)
%sampledCommandDerivative Calculate command rate on controller grid.

time = double(time(:));
command = double(command(:));

gridStart = ceil(time(1)/sampleTime)*sampleTime;
gridEnd = floor(time(end)/sampleTime)*sampleTime;

sampleGrid = (gridStart:sampleTime:gridEnd)';

if numel(sampleGrid) < 2
    rateAtTime = zeros(size(time));
    return
end

% Sample the held controller command on its discrete grid.
sampledCommand = interp1( ...
    time, command, sampleGrid, "linear");

sampledRate = [ ...
    0;
    diff(sampledCommand)/sampleTime];

% Map the sampled rate back to the logged time vector for plotting and
% evaluation using a zero-order hold.
rateAtTime = interp1( ...
    sampleGrid, sampledRate, time, ...
    "previous", "extrap");
end