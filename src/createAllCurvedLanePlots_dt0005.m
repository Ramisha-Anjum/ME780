function createAllCurvedLanePlots_dt0005
% createAllCurvedLanePlots_dt0005
% Creates all 11 report/presentation plots for the CarSim curved-lane case.
%
% Expected saved SDI session:
%   results/CarSim_CurvedLane60_mu090_dt0005/
%   CurvedLane60_mu090_dt0005_Baseline_AFS_TV.mldatx
%
% Expected run names contain:
%   Baseline, AFS, TV
%
% Recommended location:
%   AFS_vs_TV-main/src/createAllCurvedLanePlots_dt0005.m
%
% Run with:
%   createAllCurvedLanePlots_dt0005

%% Resolve project and result folders
scriptFile = string(mfilename("fullpath"));
scriptFolder = string(fileparts(scriptFile));

projectRootCandidate = string(fileparts(scriptFolder));
if isfolder(fullfile(projectRootCandidate, "results"))
    projectRoot = projectRootCandidate;
else
    projectRoot = ...
        "\\ecfile1.uwaterloo.ca\r3anjum\My Documents\ME780 Project\" + ...
        "AFS_vs_TV-main";
end

resultsDir = fullfile(projectRoot, ...
    "results", "CarSim_CurvedLane60_mu090_dt0005");
archiveFile = fullfile(resultsDir, ...
    "CurvedLane60_mu090_dt0005_Baseline_AFS_TV.mldatx");

if ~isfolder(resultsDir)
    mkdir(resultsDir);
end

%% Load saved SDI session when available
if isfile(archiveFile)
    Simulink.sdi.clear;
    Simulink.sdi.load(archiveFile);
    fprintf("Loaded SDI session:\n  %s\n", archiveFile);
else
    fprintf(["Saved SDI session was not found at:\n  %s\n" + ...
        "Using the runs currently loaded in Simulation Data Inspector.\n"], ...
        archiveFile);
end

%% Locate Baseline, AFS, and TV runs
runIDs = Simulink.sdi.getAllRunIDs;
if isempty(runIDs)
    error(["No SDI runs are loaded. Load the saved .mldatx file, " + ...
        "then run this function again."]);
end

controllerTokens = ["Baseline", "AFS", "TV"];
displayNames = ["Baseline", ...
    "Active Front Steering (AFS)", ...
    "Torque Vectoring (TV)"];

runData = repmat(emptyRunData(), 1, 3);
for k = 1:3
    runObj = findBestRun(runIDs, controllerTokens(k));
    runData(k) = extractRunData(runObj, displayNames(k));
    fprintf("Using SDI run: %s\n", runObj.Name);
end

baseline = runData(1);
afsRun = runData(2);
tvRun = runData(3);

%% Common appearance
colors = lines(3);
referenceStyle = "k--";
lineWidth = 1.8;
referenceWidth = 1.6;
fontSize = 11;
showFigures = "on";  % Change to "off" for silent export.

%% CL01: Vehicle trajectory
fig = newFigure(showFigures);
plot(baseline.XRef, baseline.YRef, referenceStyle, ...
    "LineWidth", referenceWidth, "DisplayName", "Reference");
hold on
for k = 1:3
    plot(runData(k).X, runData(k).Y, ...
        "LineWidth", lineWidth, ...
        "Color", colors(k,:), ...
        "DisplayName", runData(k).name);
end
axis equal
grid on
box on
xlabel("Longitudinal position, X [m]")
ylabel("Lateral position, Y [m]")
title("Curved-Lane Tracking at 60 km/h, \mu = 0.90: Vehicle Trajectory", ...
    "Interpreter", "tex")
legend("Location", "best", "Interpreter", "none")
applyAxesStyle(gca, fontSize)
savePlot(fig, resultsDir, "CL01_Trajectory");

%% CL02: Cross-track error
fig = newFigure(showFigures);
hold on
for k = 1:3
    plot(runData(k).time, runData(k).crossTrackError, ...
        "LineWidth", lineWidth, ...
        "Color", colors(k,:), ...
        "DisplayName", runData(k).name);
end
yline(0, "k--", "LineWidth", 0.8, "HandleVisibility", "off");
grid on
box on
xlim([0 10])
xlabel("Time [s]")
ylabel("Cross-track error, e_y [m]", "Interpreter", "tex")
title("Curved-Lane Tracking: Cross-Track Error")
legend("Location", "best", "Interpreter", "none")
applyAxesStyle(gca, fontSize)
savePlot(fig, resultsDir, "CL02_CrossTrack_Error");

%% CL03: Yaw-rate response
fig = newFigure(showFigures);
plot(baseline.time, baseline.rRef, referenceStyle, ...
    "LineWidth", referenceWidth, "DisplayName", "Reference");
hold on
for k = 1:3
    plot(runData(k).time, runData(k).r, ...
        "LineWidth", lineWidth, ...
        "Color", colors(k,:), ...
        "DisplayName", runData(k).name);
end
grid on
box on
xlim([0 10])
xlabel("Time [s]")
ylabel("Yaw rate, r [rad/s]")
title("Curved-Lane Tracking: Yaw-Rate Response")
legend("Location", "best", "Interpreter", "none")
applyAxesStyle(gca, fontSize)
savePlot(fig, resultsDir, "CL03_Yaw_Rate");

%% CL04: Heading error
fig = newFigure(showFigures);
hold on
for k = 1:3
    plot(runData(k).time, rad2deg(runData(k).headingError), ...
        "LineWidth", lineWidth, ...
        "Color", colors(k,:), ...
        "DisplayName", runData(k).name);
end
yline(0, "k--", "LineWidth", 0.8, "HandleVisibility", "off");
grid on
box on
xlim([0 10])
xlabel("Time [s]")
ylabel("Heading error, e_\psi [deg]", "Interpreter", "tex")
title("Curved-Lane Tracking: Heading Error")
legend("Location", "best", "Interpreter", "none")
applyAxesStyle(gca, fontSize)
savePlot(fig, resultsDir, "CL04_Heading_Error");

%% CL05: Vehicle sideslip
fig = newFigure(showFigures);
hold on
for k = 1:3
    plot(runData(k).time, runData(k).beta, ...
        "LineWidth", lineWidth, ...
        "Color", colors(k,:), ...
        "DisplayName", runData(k).name);
end
yline(0, "k--", "LineWidth", 0.8, "HandleVisibility", "off");
grid on
box on
xlim([0 10])
xlabel("Time [s]")
ylabel("Sideslip angle, \beta [rad]", "Interpreter", "tex")
title("Curved-Lane Tracking: Vehicle Sideslip")
legend("Location", "best", "Interpreter", "none")
applyAxesStyle(gca, fontSize)
savePlot(fig, resultsDir, "CL05_Sideslip");

%% CL06: Lateral acceleration
fig = newFigure(showFigures);
hold on
for k = 1:3
    plot(runData(k).time, runData(k).ay, ...
        "LineWidth", lineWidth, ...
        "Color", colors(k,:), ...
        "DisplayName", runData(k).name);
end
yline(0, "k--", "LineWidth", 0.8, "HandleVisibility", "off");
grid on
box on
xlim([0 10])
xlabel("Time [s]")
ylabel("Lateral acceleration, a_y [m/s^2]", "Interpreter", "tex")
title("Curved-Lane Tracking: Lateral Acceleration")
legend("Location", "best", "Interpreter", "none")
applyAxesStyle(gca, fontSize)
savePlot(fig, resultsDir, "CL06_Lateral_Acceleration");

%% CL07: Total front steering
fig = newFigure(showFigures);
hold on
for k = 1:3
    plot(runData(k).time, runData(k).deltaF, ...
        "LineWidth", lineWidth, ...
        "Color", colors(k,:), ...
        "DisplayName", runData(k).name);
end
yline(0, "k--", "LineWidth", 0.8, "HandleVisibility", "off");
grid on
box on
xlim([0 10])
xlabel("Time [s]")
ylabel("Total front steering, \delta_f [rad]", "Interpreter", "tex")
title("Curved-Lane Tracking: Total Front Steering")
legend("Location", "best", "Interpreter", "none")
applyAxesStyle(gca, fontSize)
savePlot(fig, resultsDir, "CL07_Total_Steering");

%% CL08: AFS control effort
fig = newFigure(showFigures);
plot(afsRun.time, afsRun.deltaBase, ...
    "LineWidth", lineWidth, ...
    "DisplayName", "Baseline steering, \delta_{base}");
hold on
plot(afsRun.time, afsRun.deltaAdd, ...
    "LineWidth", lineWidth, ...
    "DisplayName", "AFS correction, \delta_{add}");
plot(afsRun.time, afsRun.deltaF, ...
    "LineWidth", lineWidth, ...
    "DisplayName", "Total steering, \delta_f");
yline(0, "k--", "LineWidth", 0.8, "HandleVisibility", "off");
grid on
box on
xlim([0 10])
xlabel("Time [s]")
ylabel("Steering angle [rad]")
title("Curved-Lane Tracking: AFS Steering Correction")
legend("Location", "best", "Interpreter", "tex")
applyAxesStyle(gca, fontSize)
savePlot(fig, resultsDir, "CL08_AFS_Control_Effort");

%% CL09: TV yaw-moment command
fig = newFigure(showFigures);
plot(tvRun.time, tvRun.Mz, ...
    "LineWidth", lineWidth, ...
    "DisplayName", "Torque Vectoring (TV)");
yline(0, "k--", "LineWidth", 0.8, "HandleVisibility", "off");
grid on
box on
xlim([0 10])
xlabel("Time [s]")
ylabel("Corrective yaw moment, M_z [N m]", "Interpreter", "tex")
title("Curved-Lane Tracking: TV Yaw-Moment Command")
legend("Location", "best", "Interpreter", "none")
applyAxesStyle(gca, fontSize)
savePlot(fig, resultsDir, "CL09_TV_Yaw_Moment");

%% CL10: Longitudinal-speed verification
fig = newFigure(showFigures);
plot(baseline.time, baseline.VxRef, referenceStyle, ...
    "LineWidth", referenceWidth, "DisplayName", "Reference");
hold on
for k = 1:3
    plot(runData(k).time, runData(k).Vx, ...
        "LineWidth", lineWidth, ...
        "Color", colors(k,:), ...
        "DisplayName", runData(k).name);
end
grid on
box on
xlim([0 10])
xlabel("Time [s]")
ylabel("Longitudinal speed, V_x [m/s]", "Interpreter", "tex")
title("Curved-Lane Tracking: Longitudinal Speed")
legend("Location", "best", "Interpreter", "none")
applyAxesStyle(gca, fontSize)
savePlot(fig, resultsDir, "CL10_Longitudinal_Speed");

%% CL11: CarSim steering-interface verification
fig = newFigure(showFigures);
plot(afsRun.time, afsRun.deltaFDeg, ...
    "LineWidth", lineWidth, ...
    "DisplayName", "Commanded front steering");
hold on
plot(afsRun.time, afsRun.steerL1Deg, ...
    "LineWidth", lineWidth, ...
    "DisplayName", "Actual left-front steering");
plot(afsRun.time, afsRun.steerR1Deg, ...
    "LineWidth", lineWidth, ...
    "DisplayName", "Actual right-front steering");
grid on
box on
xlim([0 10])
xlabel("Time [s]")
ylabel("Front-wheel steering angle [deg]")
title("CarSim Curved-Lane Steering Interface Verification")
legend("Location", "best", "Interpreter", "none")
applyAxesStyle(gca, fontSize)
savePlot(fig, resultsDir, "CL11_Steering_Verification");

%% Save numerical summary for the report
metrics = createMetricsTable(runData);
writetable(metrics, fullfile(resultsDir, ...
    "CurvedLane60_dt0005_metrics.csv"));

fprintf("\nCreated all 11 curved-lane plots in:\n  %s\n", resultsDir);
fprintf("Also saved editable .fig files and CurvedLane60_dt0005_metrics.csv.\n\n");
disp(metrics)

end

%% ========================================================================
function data = extractRunData(runObj, displayName)
[time, X] = readSignalFlexible(runObj, "X", "Plant");

Y        = sampleSignalFlexible(runObj, "Y",         time, "Plant");
psi      = sampleSignalFlexible(runObj, "psi",       time, "Plant");
Vx       = sampleSignalFlexible(runObj, "Vx",        time, "Plant");
r        = sampleSignalFlexible(runObj, "r",         time, "Plant");
beta     = sampleSignalFlexible(runObj, "beta",      time, "Plant");
ay       = sampleSignalFlexible(runObj, "ay",        time, "Plant");
XRef     = sampleSignalFlexible(runObj, "X_ref",     time, "Ref");
YRef     = sampleSignalFlexible(runObj, "Y_ref",     time, "Ref");
psiRef   = sampleSignalFlexible(runObj, "psi_ref",   time, "Ref");
kappaRef = sampleSignalFlexible(runObj, "kappa_ref", time, "Ref");
VxRef    = sampleSignalFlexible(runObj, "Vx_ref",    time, "Ref");
mu       = sampleSignalFlexible(runObj, "mu",        time, "Ref");
deltaF   = sampleSignalFlexible(runObj, "Controller:1", ...
    time, "Controller");

deltaBase = readOptionalSignal(runObj, ...
    "delta_base", time, "BaseSteering", NaN);
if all(isnan(deltaBase))
    deltaBase = readOptionalSignal(runObj, ...
        "BaseSteering:1", time, "BaseSteering", NaN);
end

deltaAdd = readOptionalSignal(runObj, ...
    "delta_add", time, "Variant", NaN);
if all(isnan(deltaAdd))
    deltaAdd = deltaF - deltaBase;
end

Mz = readOptionalSignal(runObj, "Mz", time, "Variant", NaN);
if all(isnan(Mz))
    Mz = readOptionalSignal(runObj, ...
        "Controller:2", time, "Controller", 0);
end

deltaFDeg = readOptionalSignal(runObj, ...
    "delta_f_deg", time, "Plant", NaN);
if all(isnan(deltaFDeg))
    deltaFDeg = rad2deg(deltaF);
end

steerL1Deg = readOptionalSignal(runObj, ...
    "carsim_actual_Steer_L1", time, "Plant", NaN);
steerR1Deg = readOptionalSignal(runObj, ...
    "carsim_actual_Steer_R1", time, "Plant", NaN);

if all(isnan(steerL1Deg)) || all(isnan(steerR1Deg))
    error(["CarSim actual left/right steering signals were not found in " + ...
        "run '%s'. Log carsim_actual_Steer_L1 and " + ...
        "carsim_actual_Steer_R1, then save the SDI session again."], ...
        runObj.Name);
end

crossTrackError = ...
    -sin(psiRef).*(X - XRef) + cos(psiRef).*(Y - YRef);
headingError = atan2(sin(psiRef - psi), cos(psiRef - psi));

speedForLimit = max(abs(Vx), 0.1);
yawRateLimit = mu .* 9.81 ./ speedForLimit;
rRefRaw = VxRef .* kappaRef;
rRef = min(max(rRefRaw, -yawRateLimit), yawRateLimit);

data = emptyRunData();
data.name = displayName;
data.time = time;
data.X = X;
data.Y = Y;
data.psi = psi;
data.Vx = Vx;
data.r = r;
data.beta = beta;
data.ay = ay;
data.XRef = XRef;
data.YRef = YRef;
data.psiRef = psiRef;
data.kappaRef = kappaRef;
data.VxRef = VxRef;
data.mu = mu;
data.deltaBase = deltaBase;
data.deltaAdd = deltaAdd;
data.deltaF = deltaF;
data.Mz = Mz;
data.deltaFDeg = deltaFDeg;
data.steerL1Deg = steerL1Deg;
data.steerR1Deg = steerR1Deg;
data.crossTrackError = crossTrackError;
data.headingError = headingError;
data.rRef = rRef;
end

function runObj = findBestRun(runIDs, controllerToken)
matchingIDs = [];
matchingDurations = [];

for n = 1:numel(runIDs)
    candidate = Simulink.sdi.getRun(runIDs(n));
    if contains(string(candidate.Name), controllerToken, ...
            "IgnoreCase", true)
        matchingIDs(end+1) = runIDs(n); %#ok<AGROW>
        matchingDurations(end+1) = estimateRunDuration(candidate); %#ok<AGROW>
    end
end

if isempty(matchingIDs)
    names = strings(numel(runIDs),1);
    for n = 1:numel(runIDs)
        names(n) = string(Simulink.sdi.getRun(runIDs(n)).Name);
    end
    error("Could not find an SDI run containing '%s'. Available runs:\n%s", ...
        controllerToken, strjoin(names, newline));
end

maxDuration = max(matchingDurations);
idx = find(matchingDurations == maxDuration, 1, "last");
runObj = Simulink.sdi.getRun(matchingIDs(idx));
end

function duration = estimateRunDuration(runObj)
duration = 0;
ids = runObj.getAllSignalIDs;
for k = 1:numel(ids)
    try
        values = Simulink.sdi.getSignal(ids(k)).Values;
        t = double(values.Time(:));
        if ~isempty(t)
            duration = max(duration, t(end) - t(1));
        end
    catch
    end
end
end

function [time, values] = readSignalFlexible(runObj, signalName, preferredGroup)
signalID = findSignalIDFlexible(runObj, signalName, preferredGroup);
signalValues = Simulink.sdi.getSignal(signalID).Values;
time = double(signalValues.Time(:));
values = double(signalValues.Data(:));
end

function values = sampleSignalFlexible(runObj, signalName, ...
        queryTime, preferredGroup)
[signalTime, signalData] = ...
    readSignalFlexible(runObj, signalName, preferredGroup);
if isscalar(signalTime)
    values = repmat(signalData, size(queryTime));
else
    values = interp1(signalTime, signalData, ...
        queryTime, "linear", "extrap");
end
end

function values = readOptionalSignal(runObj, signalName, ...
        queryTime, preferredGroup, fallback)
try
    values = sampleSignalFlexible(runObj, ...
        signalName, queryTime, preferredGroup);
catch
    values = repmat(fallback, size(queryTime));
end
end

function signalID = findSignalIDFlexible(runObj, signalName, preferredGroup)
ids = runObj.getAllSignalIDs;
if isempty(ids)
    error("Run '%s' contains no logged signals.", runObj.Name);
end

requested = lower(string(signalName));
matches = [];
searchTexts = strings(0,1);
exactName = false(0,1);

for k = 1:numel(ids)
    sig = Simulink.sdi.getSignal(ids(k));
    nameText = "";
    pathText = "";
    sourceText = "";
    try, nameText = string(sig.Name); end %#ok<TRYNC>
    try, pathText = string(sig.BlockPath); end %#ok<TRYNC>
    try, sourceText = string(sig.DataSource); end %#ok<TRYNC>

    combined = strjoin([nameText, pathText, sourceText], " | ");
    combinedLower = lower(combined);

    isExactName = lower(nameText) == requested;
    isContained = contains(combinedLower, requested);

    if isExactName || isContained
        matches(end+1) = ids(k); %#ok<AGROW>
        searchTexts(end+1) = combined; %#ok<AGROW>
        exactName(end+1) = isExactName; %#ok<AGROW>
    end
end

if isempty(matches)
    error("Signal '%s' was not found in run '%s'.", ...
        signalName, runObj.Name);
end

if any(exactName)
    matches = matches(exactName);
    searchTexts = searchTexts(exactName);
end

if nargin >= 3 && strlength(preferredGroup) > 0
    groupMatch = contains(lower(searchTexts), ...
        lower(string(preferredGroup)));
    if any(groupMatch)
        matches = matches(groupMatch);
    end
end

signalID = matches(1);
end

function fig = newFigure(visibility)
fig = figure("Visible", visibility, "Color", "w", ...
    "Position", [100 100 1050 560]);
end

function applyAxesStyle(ax, fontSize)
set(ax, "FontSize", fontSize, "LineWidth", 0.9, ...
    "Color", "w", "XColor", "k", "YColor", "k", ...
    "GridColor", [0.78 0.78 0.78]);
end

function savePlot(fig, resultsDir, baseName)
exportgraphics(fig, fullfile(resultsDir, baseName + ".png"), ...
    "Resolution", 300);
savefig(fig, fullfile(resultsDir, baseName + ".fig"));
end

function metrics = createMetricsTable(runData)
evaluationStart = 3.0;
evaluationEnd = 10.0;

controller = strings(3,1);
crossTrackRMSE_m = zeros(3,1);
crossTrackPeak_m = zeros(3,1);
headingRMSE_deg = zeros(3,1);
headingPeak_deg = zeros(3,1);
yawRateRMSE_rad_s = zeros(3,1);
yawRatePeak_rad_s = zeros(3,1);
betaPeak_deg = zeros(3,1);
ayPeak_m_s2 = zeros(3,1);
steeringPeak_deg = zeros(3,1);
MzPeak_Nm = zeros(3,1);

for k = 1:3
    mask = runData(k).time >= evaluationStart & ...
           runData(k).time <= evaluationEnd;

    controller(k) = runData(k).name;
    crossTrackRMSE_m(k) = sqrt(mean( ...
        runData(k).crossTrackError(mask).^2));
    crossTrackPeak_m(k) = max(abs( ...
        runData(k).crossTrackError(mask)));
    headingRMSE_deg(k) = rad2deg(sqrt(mean( ...
        runData(k).headingError(mask).^2)));
    headingPeak_deg(k) = rad2deg(max(abs( ...
        runData(k).headingError(mask))));
    yawError = runData(k).rRef(mask) - runData(k).r(mask);
    yawRateRMSE_rad_s(k) = sqrt(mean(yawError.^2));
    yawRatePeak_rad_s(k) = max(abs(yawError));
    betaPeak_deg(k) = rad2deg(max(abs(runData(k).beta(mask))));
    ayPeak_m_s2(k) = max(abs(runData(k).ay(mask)));
    steeringPeak_deg(k) = rad2deg(max(abs(runData(k).deltaF(mask))));
    MzPeak_Nm(k) = max(abs(runData(k).Mz(mask)));
end

metrics = table(controller, ...
    repmat(evaluationStart,3,1), repmat(evaluationEnd,3,1), ...
    crossTrackRMSE_m, crossTrackPeak_m, ...
    headingRMSE_deg, headingPeak_deg, ...
    yawRateRMSE_rad_s, yawRatePeak_rad_s, ...
    betaPeak_deg, ayPeak_m_s2, steeringPeak_deg, MzPeak_Nm, ...
    "VariableNames", { ...
    "Controller", "EvaluationStart_s", "EvaluationEnd_s", ...
    "CrossTrackRMSE_m", "CrossTrackPeak_m", ...
    "HeadingRMSE_deg", "HeadingPeak_deg", ...
    "YawRateRMSE_rad_s", "YawRatePeak_rad_s", ...
    "BetaPeak_deg", "AyPeak_m_s2", ...
    "SteeringPeak_deg", "MzPeak_Nm"});
end

function data = emptyRunData()
data = struct( ...
    "name", "", "time", [], ...
    "X", [], "Y", [], "psi", [], "Vx", [], "r", [], ...
    "beta", [], "ay", [], ...
    "XRef", [], "YRef", [], "psiRef", [], ...
    "kappaRef", [], "VxRef", [], "mu", [], ...
    "deltaBase", [], "deltaAdd", [], ...
    "deltaF", [], "Mz", [], ...
    "deltaFDeg", [], "steerL1Deg", [], "steerR1Deg", [], ...
    "crossTrackError", [], "headingError", [], "rRef", []);
end
