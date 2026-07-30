function createDLC03CrossTrackError_v2
% Creates the cross-track-error comparison for the CarSim DLC runs.
%
% Required SDI run names (or names containing these tokens):
%   Baseline
%   AFS
%   TV
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

controllerTokens = ["Baseline", "AFS", "TV"];
displayNames = ["Baseline", "Active Front Steering (AFS)", "Torque Vectoring (TV)"];

runData = repmat(struct("name","","time",[],"crossTrackError",[]), 1, numel(controllerTokens));

for k = 1:numel(controllerTokens)
    runObj = findRunFlexible(runIDs, controllerTokens(k));

    % Read plant and reference signals using flexible matching
    [time, X] = readSignalFlexible(runObj, "X", "Plant");
    Y      = sampleSignalFlexible(runObj, "Y",       time, "Plant");
    psiRef = sampleSignalFlexible(runObj, "psi_ref", time, "Ref");
    XRef   = sampleSignalFlexible(runObj, "X_ref",   time, "Ref");
    YRef   = sampleSignalFlexible(runObj, "Y_ref",   time, "Ref");

    % Signed cross-track error in the reference-path normal direction
    crossTrackError = ...
        -sin(psiRef).*(X - XRef) + ...
         cos(psiRef).*(Y - YRef);

    runData(k).name = displayNames(k);
    runData(k).time = time;
    runData(k).crossTrackError = crossTrackError;
end

%% Plot
figureHandle = figure("Color","w","Position",[100 100 1050 560]);
hold on
plotColors = lines(3);

for k = 1:numel(runData)
    plot(runData(k).time, runData(k).crossTrackError, ...
        "LineWidth", 1.7, ...
        "Color", plotColors(k,:), ...
        "DisplayName", runData(k).name);
end

yline(0,"k--","LineWidth",0.8,"HandleVisibility","off");
grid on
box on
xlim([0 12])

xlabel("Time [s]")
ylabel("Cross-track error, e_y [m]", "Interpreter","tex")
title("Double Lane Change at 70 km/h, \mu = 0.90: Cross-Track Error", ...
    "Interpreter","tex")
legend("Location","best","Interpreter","none")
set(gca,"FontSize",11,"LineWidth",0.9)

%% Save
pngFile = fullfile(resultsDir, "DLC03_CrossTrack_Error.png");
figFile = fullfile(resultsDir, "DLC03_CrossTrack_Error.fig");

exportgraphics(figureHandle, pngFile, "Resolution", 300);
savefig(figureHandle, figFile);

fprintf("\nCross-track-error figure saved to:\n  %s\n", pngFile);

%% Metrics
evaluationStart = 30/(70/3.6);
evaluationEnd   = (145 + 20)/(70/3.6);

fprintf("\nEvaluation interval: %.6f to %.6f s\n", evaluationStart, evaluationEnd);

for k = 1:numel(runData)
    mask = runData(k).time >= evaluationStart & runData(k).time <= evaluationEnd;
    errorValues = runData(k).crossTrackError(mask);

    errorRMSE = sqrt(mean(errorValues.^2));
    errorPeak = max(abs(errorValues));

    fprintf("%-30s RMSE = %.5f m, peak = %.5f m\n", ...
        runData(k).name, errorRMSE, errorPeak);
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

    % Gather a few searchable text fields
    texts = strings(0,1);
    try, texts(end+1) = string(sig.Name); end %#ok<TRYNC>
    try, texts(end+1) = string(sig.BlockPath); end %#ok<TRYNC>
    try, texts(end+1) = string(sig.DataSource); end %#ok<TRYNC>

    texts = texts(strlength(texts) > 0);
    searchable = join(texts, " | ");
    searchableLower = lower(searchable);
    leafLower = lower(signalLeafName);

    % Flexible leaf-name matching
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

% Prefer the requested group if possible
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

% Prefer exact leaf-token matches over broad substring matches
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
