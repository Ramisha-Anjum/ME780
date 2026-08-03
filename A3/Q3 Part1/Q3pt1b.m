%% ME 780 Assignment 3 - Part 1(b)


clear; clc; close all;

%% Vehicle and suspension parameters
p.m_s   = 244.23;       % Front sprung mass per corner [kg]
p.m_u   = 25;           % Unsprung mass per wheel [kg]
p.c_s   = 800;          % Front damping coefficient [N*s/m]
p.k_s   = 13947;        % Front suspension stiffness [N/m]
p.k_t   = 150000;       % Tire vertical stiffness [N/m]
p.k_eq  = p.k_s*p.k_t/(p.k_s + p.k_t); % 1-DOF equivalent stiffness [N/m]

%% Bump parameters
p.H         = 0.008; % Bump height [m]
p.lambda    = 1.0;   % Bump length [m]

speeds = [10, 50];
t_end = 3.0;
output_step = 5e-4;

summary_rows = cell(2*numel(speeds),8);

ode_options = odeset( ...
    'RelTol',1e-8, ...
    'AbsTol',1e-10, ...
    'MaxStep',output_step);

for idx = 1:numel(speeds)
    speed = speeds(idx);
    U = speed / 3.6; % convert to m/s
    bump_time = p.lambda / U;

    x0_1dof = [0;0];
    x0_2dof = [0;0;0;0];

rhs_1dof = @(t,x) oneDofRhs(t,x,U,p);
    rhs_2dof = @(t,x) twoDofRhs(t,x,U,p);

    % Integrate before and after the bump separately so the solver stops at
    % the end of the piecewise half-sine road input.
    [t_1,x_1] = integratePiecewise(rhs_1dof,x0_1dof,bump_time,t_end,output_step,ode_options);

    [t_2,x_2] = integratePiecewise(rhs_2dof,x0_2dof,bump_time,t_end,output_step,ode_options);

    % Road displacement and velocity on the two solver time grids.
    [y_1,y_dot_1] = halfSineRoad(t_1,U,p.H,p.lambda);
    [y_2,~] = halfSineRoad(t_2,U,p.H,p.lambda);

    %% Recover physical outputs
    % 1-DOF outputs
    x_s_1 = x_1(:,1);
    x_s_dot_1 = x_1(:,2);
    x_s_ddot_1 = ( ...
        -p.c_s*(x_s_dot_1-y_dot_1) ...
        -p.k_eq*(x_s_1-y_1))/p.m_s;

    % 2-DOF outputs
    x_s_2 = x_2(:,1);
    x_s_dot_2 = x_2(:,2);
    x_u_2 = x_2(:,3);
    x_u_dot_2 = x_2(:,4);

    x_s_ddot_2 = ( ...
        -p.c_s*(x_s_dot_2-x_u_dot_2) ...
        -p.k_s*(x_s_2-x_u_2))/p.m_s;

    x_u_ddot_2 = ( ...
        -p.c_s*(x_u_dot_2-x_s_dot_2) ...
        -p.k_s*(x_u_2-x_s_2) ...
        -p.k_t*(x_u_2-y_2))/p.m_u;

    suspension_travel = x_u_2-x_s_2;
    tire_deflection = x_u_2-y_2;

   %% Plot results for the current speed

% Consistent model colours
color_1dof = [0 0.4470 0.7410];       % Blue
color_2dof = [0.8500 0.3250 0.0980];  % Red
color_road = [0 0 0];                 % Black

fig = figure('Color','w','Position',[80 60 1150 850]);
layout = tiledlayout(fig,3,2, ...
    'TileSpacing','compact','Padding','compact');

% Sprung-mass displacement comparison
ax1 = nexttile(layout);
plot(ax1,t_1,1000*y_1,':', ...
    'Color',color_road,'LineWidth',1.6);
hold(ax1,'on');

plot(ax1,t_1,1000*x_s_1,'-', ...
    'Color',color_1dof,'LineWidth',1.8);

plot(ax1,t_2,1000*x_s_2,'-', ...
    'Color',color_2dof,'LineWidth',1.8);

grid(ax1,'on');
box(ax1,'on');
ylabel(ax1,'Displacement [mm]');
title(ax1,'Sprung-Mass Displacement');
legend(ax1,{'Road','1-DOF $x_s$','2-DOF $x_s$'}, ...
    'Interpreter','latex','Location','best');

% Sprung-mass acceleration comparison
ax2 = nexttile(layout);
plot(ax2,t_1,x_s_ddot_1,'-', ...
    'Color',color_1dof,'LineWidth',1.8);
hold(ax2,'on');

plot(ax2,t_2,x_s_ddot_2,'-', ...
    'Color',color_2dof,'LineWidth',1.8);

grid(ax2,'on');
box(ax2,'on');
ylabel(ax2,'Acceleration [m/s$^2$]','Interpreter','latex');
title(ax2,'Sprung-Mass Acceleration');
legend(ax2,{'1-DOF','2-DOF'},'Location','best');

% Unsprung-mass displacement: 2-DOF only
ax3 = nexttile(layout);
plot(ax3,t_2,1000*y_2,':', ...
    'Color',color_road,'LineWidth',1.6);
hold(ax3,'on');

plot(ax3,t_2,1000*x_u_2,'-', ...
    'Color',color_2dof,'LineWidth',1.8);

grid(ax3,'on');
box(ax3,'on');
ylabel(ax3,'Displacement [mm]');
title(ax3,'2-DOF Unsprung-Mass Displacement');
legend(ax3,{'Road','$x_u$'}, ...
    'Interpreter','latex','Location','best');

% Unsprung-mass acceleration: 2-DOF only
ax4 = nexttile(layout);
plot(ax4,t_2,x_u_ddot_2,'-', ...
    'Color',color_2dof,'LineWidth',1.8);

grid(ax4,'on');
box(ax4,'on');
ylabel(ax4,'Acceleration [m/s$^2$]','Interpreter','latex');
title(ax4,'2-DOF Unsprung-Mass Acceleration');

% Suspension travel: 2-DOF only
ax5 = nexttile(layout);
plot(ax5,t_2,1000*suspension_travel,'-', ...
    'Color',color_2dof,'LineWidth',1.8);

grid(ax5,'on');
box(ax5,'on');
xlabel(ax5,'Time [s]');
ylabel(ax5,'$x_u-x_s$ [mm]','Interpreter','latex');
title(ax5,'Suspension (Wheel) Travel');

% Tire deflection: 2-DOF only
ax6 = nexttile(layout);
plot(ax6,t_2,1000*tire_deflection,'-', ...
    'Color',color_2dof,'LineWidth',1.8);

grid(ax6,'on');
box(ax6,'on');
xlabel(ax6,'Time [s]');
ylabel(ax6,'$x_u-y$ [mm]','Interpreter','latex');
title(ax6,'Tire Deflection');

title(layout,sprintf([ ...
    'Front Quarter-Car Half-Sine Bump Response: %g km/h ' ...
    '($U=%.3f$ m/s, $T_b=%.3f$ s)'], ...
    speed,U,bump_time),'Interpreter','latex');

    %% Store peak-response values
    summary_rows(end+1,:) = { ...
        speed,'1-DOF', ...
        max(abs(x_s_1)), ...
        max(abs(x_s_ddot_1)), ...
        NaN,NaN,NaN,NaN}; %#ok<SAGROW>

    summary_rows(end+1,:) = { ...
        speed,'2-DOF', ...
        max(abs(x_s_2)), ...
        max(abs(x_s_ddot_2)), ...
        max(abs(x_u_2)), ...
        max(abs(x_u_ddot_2)), ...
        max(abs(suspension_travel)), ...
        max(abs(tire_deflection))}; %#ok<SAGROW>
end


function dx = oneDofRhs(t, x, U, p)
    [y,y_dot] = halfSineRoad(t,U,p.H,p.lambda);

    x_s = x(1);
    x_s_dot = x(2);

    x_s_ddot = ( ...
        -p.c_s*(x_s_dot-y_dot) ...
        -p.k_eq*(x_s-y))/p.m_s;

    dx = [x_s_dot; x_s_ddot];
end

function dx = twoDofRhs(t,x,U,p)
    [y,~] = halfSineRoad(t,U,p.H,p.lambda);

    x_s = x(1);
    x_s_dot = x(2);
    x_u = x(3);
    x_u_dot = x(4);

    x_s_ddot = ( ...
        -p.c_s*(x_s_dot-x_u_dot) ...
        -p.k_s*(x_s-x_u))/p.m_s;

    x_u_ddot = ( ...
        -p.c_s*(x_u_dot-x_s_dot) ...
        -p.k_s*(x_u-x_s) ...
        -p.k_t*(x_u-y))/p.m_u;

    dx = [x_s_dot; x_s_ddot; x_u_dot; x_u_ddot];
end

function [y,y_dot] = halfSineRoad(t,U,H,lambda)
    bump_time = lambda/U;

    y = zeros(size(t));
    y_dot = zeros(size(t));

    on_bump = (t >= 0) & (t < bump_time);
    phase = pi*U*t(on_bump)/lambda;

    y(on_bump) = H*sin(phase);
    y_dot(on_bump) = H*pi*U/lambda*cos(phase);
end

function [t,x] = integratePiecewise( ...
    rhs,x0,bump_time,t_end,output_step,ode_options)

    t_before = unique([ ...
        (0:output_step:bump_time)'; ...
        bump_time]);

    t_after = unique([ ...
        bump_time; ...
        (bump_time+output_step:output_step:t_end)'; ...
        t_end]);

    [t_before,x_before] = ode45(rhs,t_before,x0,ode_options);
    [t_after,x_after] = ode45(rhs,t_after,x_before(end,:).',ode_options);

    t = [t_before; t_after(2:end)];
    x = [x_before; x_after(2:end,:)];
end