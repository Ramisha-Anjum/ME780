function createCurvedLaneErrorPlots
% createCurvedLaneErrorPlots
% Creates the two missing curved-lane plots:
%   CL02_CrossTrack_Error.png
%   CL04_Heading_Error.png
%
% It uses the saved SDI archive if it exists; otherwise it uses whatever
% runs are currently loaded in Simulation Data Inspector.
%
% Expected run names contain these tokens:
%   Baseline
%   AFS
%   TV
%
% Output folder:
%   AFS_vs_TV-main/results/CarSim_CurvedLane60_mu090

clearvars -except scenario

%% Output folder and archive file
projectRoot = ...
    "\\ecfile1.uwaterloo.ca\r3anjum\My Documents\ME780 Project\AFS_vs_TV-main";
resultsDir = fullfile(projectRoot, "results", "CarSim_CurvedLane60_mu090");
archiveFile = fullfile(resultsDir, "CurvedLane60_mu090_Baseline_AFS_TV.mldatx");

if ~isfolder(resultsDir)
    mkdir(resultsDir);
end

%% Load SDI archive if available
if exist(archiveFile, "file") == 2
    try
        Simulink.sdi.clear;
        Simulink.sdi.load(archiveFile);
        fprintf("Loaded SDI archive:\n  %s\n", archiveFile);
    catch ME
        warning("Could not load SDI archive automatically. Using currently loaded SDI runs instead.\n%s", ME.message);
    end
end

%% Locate runs
runIDs = Simulink.sdi.getAllRunIDs;
if isempty(runIDs)
    error([ ...
        "No runs are loaded in Simulation Data Inspector.\n" + ...
        "Either load the archive manually or keep the Baseline/AFS/TV runs in SDI."]);
end

controllerTokens = ["Baseline", "AFS", "TV"];
displayNames = ["Baseline", "Active Front Steering (AFS)", "Torque Vectoring (TV)"];
plotColors = lines(3);

runData = repmat(struct( ...
    "name", "", ...
    "time", [], ...
    "crossTrackError", [], ...
    "headingErrorDeg", []), 1, numel(controllerTokens));

for k = 1:numel(controllerTokens)
    runObj = findRunFlexible(runIDs, controllerTokens(k));

    [time, X] = readSignalFlexible(runObj, "X", "Plant");
    Y      = sampleSignalFlexible(runObj, "Y",       time, "Plant");
    psi    = sampleSignalFlexible(runObj, "psi",     time, "Plant");
    psiRef = sampleSignalFlexible(runObj, "psi_ref", time, "Ref");
    XRef   = sampleSignalFlexible(runObj, "X_ref",   time, "Ref");
    YRef   = sampleSignalFlexible(runObj, "Y_ref",   time, "Ref");

    % Signed cross-track error in the normal direction of the reference path
    crossTrackError = ...
        -sin(psiRef).*(X - XRef) + ...
         cos(psiRef).*(Y - YRef);

    % Wrapped heading error: e_psi = psi_ref - psi
    headingError = atan2(sin(psiRef - psi), cos(psiRef - psi));
    headingErrorDeg = rad2deg(headingError);

    runData(k).name = displayNames(k);
    runData(k).time = time;
    runData(k).crossTrackError = crossTrackError;
    runData(k).headingErrorDeg = headingErrorDeg;
end

%% CL02: Cross-track error
fig1 = figure("Color", "w", "Position", [100 100 1050 560]);
hold on
for k = 1:numel(runData)
    plot(runData(k).time, runData(k).crossTrackError, ...
        "LineWidth", 1.8, ...
        "Color", plotColors(k,:), ...
        "DisplayName", runData(k).name);
end

yline(0, "k--", "LineWidth", 0.9, "HandleVisibility", "off");
grid on
box on
xlim([0 10])
xlabel("Time [s]")
ylabel("Cross-track error, e_y [m]", "Interpreter", "tex")
title("Curved-Lane Tracking: Cross-Track Error", "Interpreter", "none")
legend("Location", "best", "Interpreter", "none")
set(gca, "FontSize", 11, "LineWidth", 0.9)

pngFile1 = fullfile(resultsDir, "CL02_CrossTrack_Error.png");
figFile1 = fullfile(resultsDir, "CL02_CrossTrack_Error.fig");
exportgraphics(fig1, pngFile1, "Resolution", 300);
savefig(fig1, figFile1);

%% CL04: Heading error
fig2 = figure("Color", "w", "Position", [120 120 1050 560]);
hold on
for k = 1:numel(runData)
    plot(runData(k).time, runData(k).headingErrorDeg, ...
        "LineWidth", 1.8, ...
        "Color", plotColors(k,:), ...
        "DisplayName", runData(k).name);
end

yline(0, "k--", "LineWidth", 0.9, "HandleVisibility", "off");
grid on
box on
xlim([0 10])
xlabel("Time [s]")
ylabel("Heading error, e_\psi [deg]", "Interpreter", "tex")
title("Curved-Lane Tracking: Heading Error", "Interpreter", "none")
legend("Location", "best", "Interpreter", "none")
set(gca, "FontSize", 11, "LineWidth", 0.9)

pngFile2 = fullfile(resultsDir, "CL04_Heading_Error.png");
figFile2 = fullfile(resultsDir, "CL04_Heading_Error.fig");
exportgraphics(fig2, pngFile2, "Resolution", 300);
savefig(fig2, figFile2);

fprintf("\nSaved figures:\n  %s\n  %s\n", pngFile1, pngFile2);

%% Print quick metrics
fprintf("\nSummary metrics:\n");
for k = 1:numel(runData)
    eyRMSE = sqrt(mean(runData(k).crossTrackError.^2));
    eyPeak = max(abs(runData(k).crossTrackError));
    epsiRMSE = sqrt(mean(runData(k).headingErrorDeg.^2));
    epsiPeak = max(abs(runData(k).headingErrorDeg));

    fprintf(["%-30s  e_y RMSE = %.5f m,  e_y peak = %.5f m,  " ...
             "e_psi RMSE = %.5f deg,  e_psi peak = %.5f deg\n"], ...
        runData(k).name, eyRMSE, eyPeak, epsiRMSE, epsiPeak);
end

end

%% ---------- Local functions ----------

function runObj = findRunFlexible(runIDs, controllerToken)
matchingIDs = [];

for n = 1:numel(runIDs)
    candidate = Simulink.sdi.getRun(runIDs(n));
    runName = string(candidate.Name);
    if contains(runName, controllerToken, "IgnoreCase", true)
        matchingIDs(end + 1) = runIDs(n); %#ok<AGROW>
    end
end

if isempty(matchingIDs)
    availableNames = strings(numel(runIDs),1);
    for n = 1:numel(runIDs)
        availableNames(n) = string(Simulink.sdi.getRun(runIDs(n)).Name);
    end
    error("Could not find a run containing '%s'. Available runs:\n%s", ...
        controllerToken, strjoin(availableNames, newline));
end

runObj = Simulink.sdi.getRun(matchingIDs(end));
end

function [time, values] = readSignalFlexible(runObj, signalLeafName, preferredGroup)
signalID = findSignalIDFlexible(runObj, signalLeafName, preferredGroup);
signalObject = Simulink.sdi.getSignal(signalID);
signalValues = signalObject.Values;

time = double(signalValues.Time(:));
values = double(signalValues.Data(:));
end

function values = sampleSignalFlexible(runObj, signalLeafName, queryTime, preferredGroup)
[signalTime, signalData] = readSignalFlexible(runObj, signalLeafName, preferredGroup);
if isscalar(signalTime)
    values = repmat(signalData, size(queryTime));
else
    values = interp1(signalTime, signalData, queryTime, "linear", "extrap");
end
end

function signalID = findSignalIDFlexible(runObj, signalLeafName, preferredGroup)
allIDs = runObj.getAllSignalIDs;
if isempty(allIDs)
    error("Run '%s' contains no signals.", runObj.Name);
end

matches = [];
matchNames = strings(0,1);

for n = 1:numel(allIDs)
    sig = Simulink.sdi.getSignal(allIDs(n));

    texts = strings(0,1);
    try, texts(end+1) = string(sig.Name); end %#ok<TRYNC>
    try, texts(end+1) = string(sig.BlockPath); end %#ok<TRYNC>
    try, texts(end+1) = string(sig.DataSource); end %#ok<TRYNC>

    texts = texts(strlength(texts) > 0);
    searchable = join(texts, " | ");
    searchableLower = lower(searchable);
    leafLower = lower(signalLeafName);

    tokens = regexp(char(searchableLower), '[^a-zA-Z0-9_]+', 'split');
    tokens = string(tokens);
    tokens = tokens(strlength(tokens) > 0);

    isLeafMatch = any(tokens == leafLower) || contains(searchableLower, leafLower);

    if isLeafMatch
        matches(end+1) = allIDs(n); %#ok<AGROW>
        matchNames(end+1) = searchable; %#ok<AGROW>
    end
end

if isempty(matches)
    fprintf("\nSignals available in run '%s':\n", runObj.Name);
    for n = 1:numel(allIDs)
        sig = Simulink.sdi.getSignal(allIDs(n));
        try
            fprintf("  %s\n", string(sig.Name));
        catch
        end
    end
    error("Signal '%s' was not found in run '%s'.", signalLeafName, runObj.Name);
end

if nargin >= 3 && strlength(preferredGroup) > 0
    keep = false(size(matches));
    for n = 1:numel(matches)
        if contains(lower(matchNames(n)), lower(preferredGroup))
            keep(n) = true;
        end
    end
    if any(keep)
        matches = matches(keep);
        matchNames = matchNames(keep);
    end
end

bestIdx = 1;
for n = 1:numel(matchNames)
    txt = lower(matchNames(n));
    tokens = regexp(char(txt), '[^a-zA-Z0-9_]+', 'split');
    tokens = string(tokens);
    tokens = tokens(strlength(tokens) > 0);
    if any(tokens == lower(signalLeafName))
        bestIdx = n;
        break
    end
end

signalID = matches(bestIdx);
end
