%% ME 780 Assignment 3 - Part 1.2(a)
% Front 1-DOF quarter-car displacement and acceleration transmissibility

clear;
clc;
close all;

%% Front quarter-car parameters
m_s = 244.23;       % Front sprung mass per corner [kg]
c_s = 800;          % Front damping coefficient per wheel [N*s/m]
k_s = 13947;        % Front suspension spring rate per wheel [N/m]
k_t = 150000;       % Tire vertical stiffness per wheel [N/m]

% Suspension and tire stiffnesses in series for the course 1-DOF model.
k_eq = (k_s*k_t)/(k_s + k_t);

omega_n = sqrt(k_eq/m_s);                 % Natural frequency [rad/s]
f_n = omega_n/(2*pi);                     % Natural frequency [Hz]
zeta = c_s/(2*sqrt(k_eq*m_s));            % Damping ratio [-]

%% Frequency sweep
r = linspace(0,10,5001);                  % r = omega/omega_n
f = r*f_n;                                % Corresponding frequency [Hz]

denominator = sqrt((1-r.^2).^2 + (2*zeta*r).^2);

% Sprung-mass displacement transmissibility |X_s/Y|.
T_x = sqrt(1 + (2*zeta*r).^2)./denominator;

% Normalized sprung-mass acceleration transmissibility used in Lecture 5:
% |X_s_ddot/(Y*omega_n^2)| = r^2*|X_s/Y|.
T_a = r.^2.*T_x;

% Locate the displacement-transmissibility peak.
[T_x_peak,r_peak_index] = max(T_x);
r_peak = r(r_peak_index);
f_peak = f(r_peak_index);

%% Plot transmissibility functions
fig = figure('Color','w','Position',[100 100 900 700]);
layout = tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');

% Displacement transmissibility
ax1 = nexttile(layout);
h_Tx = plot(ax1,r,T_x,'LineWidth',2.0,'Color',[0 0.36 0.67]);
hold(ax1,'on');
xline(ax1,1,'--k','$r=1$','Interpreter','latex', ...
    'LabelVerticalAlignment','bottom','LineWidth',1.1, ...
    'HandleVisibility','off');
xline(ax1,sqrt(2),':','Isolation threshold, $r=\sqrt{2}$', ...
    'Interpreter','latex','LabelVerticalAlignment','bottom','LineWidth',1.1, ...
    'HandleVisibility','off');
h_peak = plot(ax1,r_peak,T_x_peak,'o','MarkerSize',7, ...
    'MarkerFaceColor',[0.85 0.20 0.15],'MarkerEdgeColor','none');
grid(ax1,'on');
box(ax1,'on');
xlim(ax1,[0 10]);
ylabel(ax1,'$T_x=|X_s/Y|$','Interpreter','latex');
title(ax1,'Front 1-DOF Sprung-Mass Displacement Transmissibility', ...
    'Interpreter','latex');
legend(ax1,[h_Tx h_peak],{'$T_x$',sprintf('Peak: $r=%.3f$, $f=%.3f$ Hz', ...
    r_peak,f_peak)},'Interpreter','latex','Location','northeast');

% Normalized acceleration transmissibility
ax2 = nexttile(layout);
plot(ax2,r,T_a,'LineWidth',2.0,'Color',[0.80 0.24 0.18]);
hold(ax2,'on');
xline(ax2,1,'--k','$r=1$','Interpreter','latex', ...
    'LabelVerticalAlignment','bottom','LineWidth',1.1, ...
    'HandleVisibility','off');
grid(ax2,'on');
box(ax2,'on');
xlim(ax2,[0 10]);
xlabel(ax2,'Frequency ratio, $r=\omega/\omega_n$', ...
    'Interpreter','latex');
ylabel(ax2,'$T_a=|\ddot{X}_s/(Y\omega_n^2)|$', ...
    'Interpreter','latex');
title(ax2,'Front 1-DOF Normalized Sprung-Mass Acceleration Transmissibility', ...
    'Interpreter','latex');

title(layout,sprintf(['Front 1-DOF Frequency Response: ' ...
    '$f_n=%.3f$ Hz, $\\zeta=%.4f$'],f_n,zeta),'Interpreter','latex');

%% Export report-ready figures
output_directory = fullfile(pwd,'figures');
if ~exist(output_directory,'dir')
    mkdir(output_directory);
end

exportgraphics(fig,fullfile(output_directory, ...
    'part2a_front_transmissibility.png'),'Resolution',300);
exportgraphics(fig,fullfile(output_directory, ...
    'part2a_front_transmissibility.pdf'),'ContentType','vector');

%% Display calculated values
fprintf('Equivalent stiffness: %.2f N/m\n',k_eq);
fprintf('Natural frequency: %.4f rad/s = %.4f Hz\n',omega_n,f_n);
fprintf('Damping ratio: %.4f\n',zeta);
fprintf('Displacement peak: T_x = %.4f at r = %.4f (f = %.4f Hz)\n', ...
    T_x_peak,r_peak,f_peak);
