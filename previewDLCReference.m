clear
clc
close all

setupProject
scenario = DLCParams();

X = linspace( ...
    0, ...
    scenario.Vx*scenario.stopTime, ...
    2000)';

nSamples = numel(X);

Yref = zeros(nSamples,1);
psiRef = zeros(nSamples,1);
kappaRef = zeros(nSamples,1);

for k = 1:nSamples
    [~, Yref(k), psiRef(k), kappaRef(k), ~, ~] = ...
        dlcReferenceFromX( ...
        X(k), ...
        scenario.Vx, ...
        scenario.mu, ...
        scenario.laneWidth, ...
        scenario.xStart, ...
        scenario.transitionLength, ...
        scenario.holdLength);
end

ayReference = scenario.Vx^2*kappaRef;

fprintf("\nDLC reference preview\n");
fprintf("Speed                  = %.3f m/s\n", scenario.Vx);
fprintf("Lane displacement      = %.3f m\n", max(Yref));
fprintf("Final lateral position = %.6f m\n", Yref(end));
fprintf("Peak path heading      = %.3f deg\n", ...
    rad2deg(max(abs(psiRef))));
fprintf("Peak path curvature    = %.6f 1/m\n", ...
    max(abs(kappaRef)));
fprintf("Approximate peak V^2k  = %.3f m/s^2\n", ...
    max(abs(ayReference)));

figure("Name","DLC reference preview","Color","w");
tiledlayout(4,1,"TileSpacing","compact");

nexttile;
plot(X,Yref,"LineWidth",1.5);
grid on;
ylabel("Y_{ref} [m]");
title("Double-lane-change path");

nexttile;
plot(X,rad2deg(psiRef),"LineWidth",1.5);
grid on;
ylabel("\psi_{ref} [deg]");
title("Reference heading");

nexttile;
plot(X,kappaRef,"LineWidth",1.5);
grid on;
ylabel("\kappa_{ref} [1/m]");
title("Reference curvature");

nexttile;
plot(X,ayReference,"LineWidth",1.5);
grid on;
xlabel("Longitudinal position X [m]");
ylabel("V_x^2\kappa [m/s^2]");
title("Kinematic lateral-acceleration reference");