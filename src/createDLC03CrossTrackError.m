%% createDLC03CrossTrackError
% Creates the cross-track-error comparison for the CarSim DLC runs.
%
% Required SDI run names:
%   CarSim_DLC70_Baseline
%   CarSim_DLC70_AFS
%   CarSim_DLC70_TV
%
% If the runs are not currently in SDI, first load the saved session:
%   Simulink.sdi.clear
%   Simulink.sdi.load("...\DLC70_mu090_Baseline_AFS_TV.mldatx")

clearvars -except scenario

%% Output folder

resultsDir = ...
    "\\ecfile1.uwaterloo.ca\r3anjum\My Documents\ME780 Project\" + ...
    "AFS_vs_TV-main\results\CarSim_DLC70_mu090";

if ~isfolder(resultsDir)
    mkdir(resultsDir);
end

%% Locate the three SDI runs

runIDs = Simulink.sdi.getAllRunIDs;

if isempty(runIDs)
    error("No runs are loaded in Simulation Data Inspector.");
end

controllerNames = ["Baseline", "AFS", "TV"];
displayNames = [ ...
    "Baseline", ...
    "Active Front Steering (AFS)", ...
    "Torque Vectoring (TV)"];

runData = repmat(struct( ...
    "name", "", ...
    "time", [], ...
    "crossTrackError", []), 1, numel(controllerNames));

for k = 1:numel(controllerNames)
    runObj = findRun(runIDs, controllerNames(k));

    % Use the plant X signal time as the common time vector for this run.
    [time, X] = readSignal(runObj, "X");

    Y      = sampleSignal(runObj, "Y",       time);
    psiRef = sampleSignal(runObj, "psi_ref", time);
    XRef   = sampleSignal(runObj, "X_ref",   time);
    YRef   = sampleSignal(runObj, "Y_ref",   time);

    % Signed cross-track error in the reference-path normal direction.
    crossTrackError = ...
        -sin(psiRef).*(X - XRef) ...
        + cos(psiRef).*(Y - YRef);

    runData(k).name = displayNames(k);
    runData(k).time = time;
    runData(k).crossTrackError = crossTrackError;
end

%% Create report-quality figure

figureHandle = figure( ...
    "Color", "w", ...
    "Position", [100 100 1050 560]);

hold on

lineStyles = ["-", "-", "-"];
lineWidths = [1.7, 1.7, 1.7];
plotColors = lines(3);

for k = 1:numel(runData)
    plot( ...
        runData(k).time, ...
        runData(k).crossTrackError, ...
        lineStyles(k), ...
        "LineWidth", lineWidths(k), ...
        "Color", plotColors(k,:), ...
        "DisplayName", runData(k).name);
end

yline(0, "k--", ...
    "LineWidth", 0.8, ...
    "HandleVisibility", "off");

grid on
box on
xlim([0 12])

xlabel("Time [s]")
ylabel("Cross-track error, e_y [m]", "Interpreter", "tex")
title( ...
    "Double Lane Change at 70 km/h, \mu = 0.90: Cross-Track Error", ...
    "Interpreter", "tex")

legend( ...
    "Location", "best", ...
    "Interpreter", "none")

set(gca, ...
    "FontSize", 11, ...
    "LineWidth", 0.9)

%% Save PNG and editable MATLAB figure

pngFile = fullfile(resultsDir, "DLC03_CrossTrack_Error.png");
figFile = fullfile(resultsDir, "DLC03_CrossTrack_Error.fig");

exportgraphics(figureHandle, pngFile, "Resolution", 300);
savefig(figureHandle, figFile);

fprintf("\nCross-track-error figure saved to:\n  %s\n", pngFile);

%% Optional numerical summary over the established evaluation interval

evaluationStart = 30/(70/3.6);
evaluationEnd = (145 + 20)/(70/3.6);

fprintf("\nEvaluation interval: %.6f to %.6f s\n", ...
    evaluationStart, evaluationEnd);

for k = 1:numel(runData)
    mask = ...
        runData(k).time >= evaluationStart ...
        & runData(k).time <= evaluationEnd;

    errorValues = runData(k).crossTrackError(mask);

    errorRMSE = sqrt(mean(errorValues.^2));
    errorPeak = max(abs(errorValues));

    fprintf( ...
        "%-30s RMSE = %.5f m, peak = %.5f m\n", ...
        runData(k).name, errorRMSE, errorPeak);
end

%% Local functions

function runObj = findRun(runIDs, controllerToken)
%findRun Find the most recent full run containing a controller token.

matchingIDs = [];

for n = 1:numel(runIDs)
    candidate = Simulink.sdi.getRun(runIDs(n));
    runName = string(candidate.Name);

    if contains(runName, controllerToken, "IgnoreCase", true)
        matchingIDs(end + 1) = runIDs(n); %#ok<AGROW>
    end
end

if isempty(matchingIDs)
    availableNames = strings(numel(runIDs), 1);

    for n = 1:numel(runIDs)
        availableNames(n) = string( ...
            Simulink.sdi.getRun(runIDs(n)).Name);
    end

    error( ...
        "Could not find a run containing '%s'. Available runs:\n%s", ...
        controllerToken, strjoin(availableNames, newline));
end

% Use the most recently created matching run.
runObj = Simulink.sdi.getRun(matchingIDs(end));
end


function [time, values] = readSignal(runObj, signalName)
%readSignal Read one uniquely named SDI signal.

signalIDs = getSignalIDsByName(runObj, signalName);

if isempty(signalIDs)
    error( ...
        "Signal '%s' was not found in run '%s'.", ...
        signalName, runObj.Name);
end

if numel(signalIDs) > 1
    error( ...
        "Signal name '%s' is not unique in run '%s'.", ...
        signalName, runObj.Name);
end

signalObject = Simulink.sdi.getSignal(signalIDs(1));
signalValues = signalObject.Values;

time = double(signalValues.Time(:));
values = double(signalValues.Data(:));
end


function values = sampleSignal(runObj, signalName, queryTime)
%sampleSignal Interpolate an SDI signal onto a requested time vector.

[signalTime, signalData] = readSignal(runObj, signalName);

if isscalar(signalTime)
    values = repmat(signalData, size(queryTime));
else
    values = interp1( ...
        signalTime, signalData, queryTime, ...
        "linear", "extrap");
end
end
