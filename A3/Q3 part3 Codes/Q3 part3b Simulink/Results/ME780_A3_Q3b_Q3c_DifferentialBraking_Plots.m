%% ME780 Assignment 3 - Part 3(b) and Part 3(c)
% Corrected differential-braking CarSim/Simulink results
%
% Expected SDI run names:
%   Mu090_OFF_DB
%   Mu090_ON_DB
%   Mu055_OFF_DB
%   Mu055_ON_DB
%
% Expected logged signals:
%   Vx, Beta, Ay, Xcg_TM, Ycg_TM
%   P_L1, P_L2, P_R1, P_R2
%   r_desired, r_measured, Mz_cmd
%
% Place this script in the same folder as:
%   A3_Q3b_Q3c_DifferentialBraking.mldatx
% and run the script. All figures and the summary table will be saved in
% a subfolder called "Generated_Plots".

clear;
clc;
close all;

%% 1. Locate and load the Simulation Data Inspector session
scriptFolder = fileparts(mfilename('fullpath'));
if isempty(scriptFolder)
    scriptFolder = pwd;
end

sessionFile = fullfile(scriptFolder, ...
    'A3_Q3b_Q3c_DifferentialBraking.mldatx');

if ~isfile(sessionFile)
    [selectedFile, selectedPath] = uigetfile('*.mldatx', ...
        'Select A3_Q3b_Q3c_DifferentialBraking.mldatx');
    if isequal(selectedFile, 0)
        error('No SDI session file was selected.');
    end
    sessionFile = fullfile(selectedPath, selectedFile);
end

outputFolder = fullfile(fileparts(sessionFile), 'Generated_Plots');
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

Simulink.sdi.clear;
Simulink.sdi.load(sessionFile);

%% 2. Retrieve the four final runs
run090Off = findRunByName("Mu090_OFF_DB");
run090On  = findRunByName("Mu090_ON_DB");
run055Off = findRunByName("Mu055_OFF_DB");
run055On  = findRunByName("Mu055_ON_DB");

fprintf('\nThe following SDI runs were loaded successfully:\n');
fprintf('  Mu090_OFF_DB\n');
fprintf('  Mu090_ON_DB\n');
fprintf('  Mu055_OFF_DB\n');
fprintf('  Mu055_ON_DB\n\n');

%% 3. Yaw-rate tracking: mu = 0.90
fig = figure('Name', 'Yaw-rate tracking - mu 0.90', 'Color', 'w');
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', ...
    'Padding', 'compact');

nexttile;
plotSDISignal(run090Off, "r_desired", ...
    'DisplayName', 'Desired yaw rate');
hold on;
plotSDISignal(run090Off, "r_measured", ...
    'DisplayName', 'Measured yaw rate');
grid on;
xlim([0 12]);
ylabel('Yaw rate [rad/s]');
title('\mu = 0.90, controller OFF');
legend('Location', 'best');

nexttile;
plotSDISignal(run090On, "r_desired", ...
    'DisplayName', 'Desired yaw rate');
hold on;
plotSDISignal(run090On, "r_measured", ...
    'DisplayName', 'Measured yaw rate');
grid on;
xlim([0 12]);
xlabel('Time [s]');
ylabel('Yaw rate [rad/s]');
title('\mu = 0.90, controller ON');
legend('Location', 'best');

title(tl, 'Yaw-Rate Tracking on the High-Friction Road');
saveFigureFiles(fig, outputFolder, 'Fig01_YawRate_Mu090');

%% 4. Yaw-rate tracking: mu = 0.55
fig = figure('Name', 'Yaw-rate tracking - mu 0.55', 'Color', 'w');
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', ...
    'Padding', 'compact');

nexttile;
plotSDISignal(run055Off, "r_desired", ...
    'DisplayName', 'Desired yaw rate');
hold on;
plotSDISignal(run055Off, "r_measured", ...
    'DisplayName', 'Measured yaw rate');
grid on;
xlim([0 12]);
ylabel('Yaw rate [rad/s]');
title('\mu = 0.55, controller OFF');
legend('Location', 'best');

nexttile;
plotSDISignal(run055On, "r_desired", ...
    'DisplayName', 'Desired yaw rate');
hold on;
plotSDISignal(run055On, "r_measured", ...
    'DisplayName', 'Measured yaw rate');
grid on;
xlim([0 12]);
xlabel('Time [s]');
ylabel('Yaw rate [rad/s]');
title('\mu = 0.55, controller ON');
legend('Location', 'best');

title(tl, 'Yaw-Rate Tracking on the Reduced-Friction Road');
saveFigureFiles(fig, outputFolder, 'Fig02_YawRate_Mu055');

%% 5. Corrective yaw-moment command
fig = figure('Name', 'Corrective yaw moment', 'Color', 'w');
plotSDISignal(run090On, "Mz_cmd", ...
    'DisplayName', '\mu = 0.90, controller ON');
hold on;
plotSDISignal(run055On, "Mz_cmd", ...
    'DisplayName', '\mu = 0.55, controller ON');
yline(2800, '--', '+2800 N m limit', 'HandleVisibility', 'off');
yline(-2800, '--', '-2800 N m limit', 'HandleVisibility', 'off');
grid on;
xlim([0 12]);
xlabel('Time [s]');
ylabel('\Delta M_z^{cmd} [N m]');
title('Requested Corrective Yaw-Moment Command');
legend('Location', 'best');
saveFigureFiles(fig, outputFolder, 'Fig03_CorrectiveYawMoment');

%% 6. Differential-brake pressures: mu = 0.90
fig = figure('Name', 'Brake pressures - mu 0.90', 'Color', 'w');
plotSDISignal(run090On, "P_L1", ...
    'DisplayName', 'Front left, P_{L1}');
hold on;
plotSDISignal(run090On, "P_L2", ...
    'DisplayName', 'Rear left, P_{L2}');
plotSDISignal(run090On, "P_R1", ...
    'DisplayName', 'Front right, P_{R1}');
plotSDISignal(run090On, "P_R2", ...
    'DisplayName', 'Rear right, P_{R2}');
grid on;
xlim([0 12]);
ylim([0 inf]);
xlabel('Time [s]');
ylabel('Brake pressure [MPa]');
title('Differential-Brake Pressure Allocation, \mu = 0.90');
legend('Location', 'best');
saveFigureFiles(fig, outputFolder, 'Fig04_BrakePressures_Mu090');

%% 7. Differential-brake pressures: mu = 0.55
fig = figure('Name', 'Brake pressures - mu 0.55', 'Color', 'w');
plotSDISignal(run055On, "P_L1", ...
    'DisplayName', 'Front left, P_{L1}');
hold on;
plotSDISignal(run055On, "P_L2", ...
    'DisplayName', 'Rear left, P_{L2}');
plotSDISignal(run055On, "P_R1", ...
    'DisplayName', 'Front right, P_{R1}');
plotSDISignal(run055On, "P_R2", ...
    'DisplayName', 'Rear right, P_{R2}');
grid on;
xlim([0 12]);
ylim([0 inf]);
xlabel('Time [s]');
ylabel('Brake pressure [MPa]');
title('Differential-Brake Pressure Allocation, \mu = 0.55');
legend('Location', 'best');
saveFigureFiles(fig, outputFolder, 'Fig05_BrakePressures_Mu055');

%% 8. Vehicle sideslip comparison
fig = figure('Name', 'Vehicle sideslip', 'Color', 'w');
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', ...
    'Padding', 'compact');

nexttile;
plotSDISignal(run090Off, "Beta", 'DisplayName', 'Controller OFF');
hold on;
plotSDISignal(run090On, "Beta", 'DisplayName', 'Controller ON');
grid on;
xlim([0 12]);
ylabel('\beta [deg]');
title('\mu = 0.90');
legend('Location', 'best');

nexttile;
plotSDISignal(run055Off, "Beta", 'DisplayName', 'Controller OFF');
hold on;
plotSDISignal(run055On, "Beta", 'DisplayName', 'Controller ON');
grid on;
xlim([0 12]);
xlabel('Time [s]');
ylabel('\beta [deg]');
title('\mu = 0.55');
legend('Location', 'best');

title(tl, 'Vehicle Sideslip-Angle Response');
saveFigureFiles(fig, outputFolder, 'Fig06_Sideslip');

%% 9. Lateral-acceleration comparison
fig = figure('Name', 'Lateral acceleration', 'Color', 'w');
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', ...
    'Padding', 'compact');

nexttile;
plotSDISignal(run090Off, "Ay", 'DisplayName', 'Controller OFF');
hold on;
plotSDISignal(run090On, "Ay", 'DisplayName', 'Controller ON');
grid on;
xlim([0 12]);
ylabel('a_y [g]');
title('\mu = 0.90');
legend('Location', 'best');

nexttile;
plotSDISignal(run055Off, "Ay", 'DisplayName', 'Controller OFF');
hold on;
plotSDISignal(run055On, "Ay", 'DisplayName', 'Controller ON');
grid on;
xlim([0 12]);
xlabel('Time [s]');
ylabel('a_y [g]');
title('\mu = 0.55');
legend('Location', 'best');

title(tl, 'Vehicle Lateral-Acceleration Response');
saveFigureFiles(fig, outputFolder, 'Fig07_LateralAcceleration');

%% 10. Longitudinal-speed verification
fig = figure('Name', 'Longitudinal speed', 'Color', 'w');
plotSDISignal(run090Off, "Vx", ...
    'DisplayName', '\mu = 0.90, controller OFF');
hold on;
plotSDISignal(run090On, "Vx", ...
    'DisplayName', '\mu = 0.90, controller ON');
plotSDISignal(run055Off, "Vx", ...
    'DisplayName', '\mu = 0.55, controller OFF');
plotSDISignal(run055On, "Vx", ...
    'DisplayName', '\mu = 0.55, controller ON');
yline(60, '--', 'Target speed', 'HandleVisibility', 'off');
grid on;
xlim([0 12]);
xlabel('Time [s]');
ylabel('V_x [km/h]');
title('Longitudinal-Speed Verification');
legend('Location', 'best');
saveFigureFiles(fig, outputFolder, 'Fig08_LongitudinalSpeed');

%% 11. Vehicle trajectory
fig = figure('Name', 'Vehicle trajectory', 'Color', 'w');
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', ...
    'Padding', 'compact');

nexttile;
plotXYSignals(run090Off, "Xcg_TM", "Ycg_TM", ...
    'DisplayName', 'Controller OFF');
hold on;
plotXYSignals(run090On, "Xcg_TM", "Ycg_TM", ...
    'DisplayName', 'Controller ON');
grid on;
axis equal;
xlabel('Longitudinal position, X [m]');
ylabel('Lateral position, Y [m]');
title('\mu = 0.90');
legend('Location', 'best');

nexttile;
plotXYSignals(run055Off, "Xcg_TM", "Ycg_TM", ...
    'DisplayName', 'Controller OFF');
hold on;
plotXYSignals(run055On, "Xcg_TM", "Ycg_TM", ...
    'DisplayName', 'Controller ON');
grid on;
axis equal;
xlabel('Longitudinal position, X [m]');
ylabel('Lateral position, Y [m]');
title('\mu = 0.55');
legend('Location', 'best');

title(tl, 'Vehicle Trajectory');
saveFigureFiles(fig, outputFolder, 'Fig09_Trajectory');

%% 12. Yaw-rate tracking error comparison
fig = figure('Name', 'Yaw-rate error', 'Color', 'w');
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', ...
    'Padding', 'compact');

nexttile;
plotYawError(run090Off, 'DisplayName', 'Controller OFF');
hold on;
plotYawError(run090On, 'DisplayName', 'Controller ON');
grid on;
xlim([0 12]);
ylabel('e_r [rad/s]');
title('\mu = 0.90');
legend('Location', 'best');

nexttile;
plotYawError(run055Off, 'DisplayName', 'Controller OFF');
hold on;
plotYawError(run055On, 'DisplayName', 'Controller ON');
grid on;
xlim([0 12]);
xlabel('Time [s]');
ylabel('e_r [rad/s]');
title('\mu = 0.55');
legend('Location', 'best');

title(tl, 'Yaw-Rate Tracking Error');
saveFigureFiles(fig, outputFolder, 'Fig10_YawRateError');

%% 13. Calculate and save summary metrics
runObjects = {run090Off; run090On; run055Off; run055On};
caseNames = ["Mu090_OFF_DB"; "Mu090_ON_DB"; ...
             "Mu055_OFF_DB"; "Mu055_ON_DB"];

nCases = numel(runObjects);
YawRMSE_rad_s          = zeros(nCases, 1);
PeakYawError_rad_s     = zeros(nCases, 1);
PeakMeasuredYaw_rad_s  = zeros(nCases, 1);
PeakBeta_deg           = zeros(nCases, 1);
PeakAy_g               = zeros(nCases, 1);
VxMin_kmh              = zeros(nCases, 1);
VxMax_kmh              = zeros(nCases, 1);
PeakMzCmd_Nm           = zeros(nCases, 1);
PeakFrontPressure_MPa  = zeros(nCases, 1);
PeakRearPressure_MPa   = zeros(nCases, 1);

for k = 1:nCases
    thisRun = runObjects{k};

    desiredTS = getSignalTimeseries(thisRun, "r_desired");
    measuredTS = getSignalTimeseries(thisRun, "r_measured");

    desired = columnData(desiredTS.Data);
    measured = interp1(measuredTS.Time, ...
        columnData(measuredTS.Data), desiredTS.Time, ...
        'linear', 'extrap');
    yawError = desired - measured;

    YawRMSE_rad_s(k) = sqrt(mean(yawError.^2, 'omitnan'));
    PeakYawError_rad_s(k) = max(abs(yawError), [], 'omitnan');
    PeakMeasuredYaw_rad_s(k) = max(abs(measured), [], 'omitnan');
    PeakBeta_deg(k) = peakAbsoluteValue(thisRun, "Beta");
    PeakAy_g(k) = peakAbsoluteValue(thisRun, "Ay");

    vxTS = getSignalTimeseries(thisRun, "Vx");
    vxData = columnData(vxTS.Data);
    VxMin_kmh(k) = min(vxData, [], 'omitnan');
    VxMax_kmh(k) = max(vxData, [], 'omitnan');

    PeakMzCmd_Nm(k) = peakAbsoluteValue(thisRun, "Mz_cmd");
    PeakFrontPressure_MPa(k) = max([ ...
        peakAbsoluteValue(thisRun, "P_L1"), ...
        peakAbsoluteValue(thisRun, "P_R1")]);
    PeakRearPressure_MPa(k) = max([ ...
        peakAbsoluteValue(thisRun, "P_L2"), ...
        peakAbsoluteValue(thisRun, "P_R2")]);
end

summaryTable = table(caseNames, YawRMSE_rad_s, ...
    PeakYawError_rad_s, PeakMeasuredYaw_rad_s, PeakBeta_deg, ...
    PeakAy_g, VxMin_kmh, VxMax_kmh, PeakMzCmd_Nm, ...
    PeakFrontPressure_MPa, PeakRearPressure_MPa, ...
    'VariableNames', {'Case', 'YawRMSE_rad_s', ...
    'PeakYawError_rad_s', 'PeakMeasuredYaw_rad_s', ...
    'PeakBeta_deg', 'PeakAy_g', 'VxMin_kmh', 'VxMax_kmh', ...
    'PeakMzCmd_Nm', 'PeakFrontPressure_MPa', ...
    'PeakRearPressure_MPa'});

disp(summaryTable);

csvFile = fullfile(outputFolder, ...
    'A3_Q3b_Q3c_SummaryMetrics.csv');
writetable(summaryTable, csvFile);

matFile = fullfile(outputFolder, ...
    'A3_Q3b_Q3c_SummaryMetrics.mat');
save(matFile, 'summaryTable');

fprintf('\nCompleted successfully.\n');
fprintf('Figures and metrics were saved in:\n%s\n', outputFolder);

%% Local functions
function runObj = findRunByName(requiredName)
    runIDs = Simulink.sdi.getAllRunIDs;
    matchedIDs = [];

    for i = 1:numel(runIDs)
        candidate = Simulink.sdi.getRun(runIDs(i));
        if string(candidate.Name) == string(requiredName)
            matchedIDs(end + 1) = runIDs(i); %#ok<AGROW>
        end
    end

    if isempty(matchedIDs)
        error('The SDI run "%s" was not found.', requiredName);
    elseif numel(matchedIDs) > 1
        error('More than one SDI run is named "%s".', requiredName);
    end

    runObj = Simulink.sdi.getRun(matchedIDs(1));
end

function ts = getSignalTimeseries(runObj, signalName)
    signalObjects = getSignalsByName(runObj, char(signalName));

    if isempty(signalObjects)
        error('Signal "%s" was not found in run "%s".', ...
            signalName, string(runObj.Name));
    end

    % Xcg_TM and Ycg_TM each appear twice in the uploaded session:
    % once from the Demux and once from the XY Graph. The first copy is
    % the Demux signal and contains the same numerical trajectory data.
    if numel(signalObjects) > 1
        signalObjects = signalObjects(1);
    end

    ts = signalObjects.Values;
    if ~isa(ts, 'timeseries')
        error('Signal "%s" is not stored as a scalar timeseries.', ...
            signalName);
    end
end

function data = columnData(dataIn)
    data = squeeze(dataIn);
    if isrow(data)
        data = data.';
    end
end

function plotSDISignal(runObj, signalName, varargin)
    ts = getSignalTimeseries(runObj, signalName);
    plot(ts.Time, columnData(ts.Data), 'LineWidth', 1.5, varargin{:});
end

function plotXYSignals(runObj, xName, yName, varargin)
    xTS = getSignalTimeseries(runObj, xName);
    yTS = getSignalTimeseries(runObj, yName);

    commonTime = unique([xTS.Time(:); yTS.Time(:)]);
    xData = interp1(xTS.Time, columnData(xTS.Data), ...
        commonTime, 'linear', 'extrap');
    yData = interp1(yTS.Time, columnData(yTS.Data), ...
        commonTime, 'linear', 'extrap');

    plot(xData, yData, 'LineWidth', 1.5, varargin{:});
end

function plotYawError(runObj, varargin)
    desiredTS = getSignalTimeseries(runObj, "r_desired");
    measuredTS = getSignalTimeseries(runObj, "r_measured");

    desired = columnData(desiredTS.Data);
    measured = interp1(measuredTS.Time, columnData(measuredTS.Data), ...
        desiredTS.Time, 'linear', 'extrap');
    yawError = desired - measured;

    plot(desiredTS.Time, yawError, 'LineWidth', 1.5, varargin{:});
end

function value = peakAbsoluteValue(runObj, signalName)
    ts = getSignalTimeseries(runObj, signalName);
    value = max(abs(columnData(ts.Data)), [], 'omitnan');
end

function saveFigureFiles(fig, folder, baseName)
    pngFile = fullfile(folder, baseName + ".png");
    figFile = fullfile(folder, baseName + ".fig");

    exportgraphics(fig, pngFile, 'Resolution', 300);
    savefig(fig, figFile);
end
