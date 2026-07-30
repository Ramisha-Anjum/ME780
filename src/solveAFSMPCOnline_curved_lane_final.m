function deltaAFS = solveAFSMPCOnline(u)
%solveAFSMPCOnline Online MPC controller for Active Front Steering.
%
% Input vector:
%   u(1) = deltaBase  [rad]
%   u(2) = beta       [rad]
%   u(3) = r          [rad/s]
%   u(4) = rRef       [rad/s]
%          steering-based desired yaw rate derived from deltaBase
%   u(5) = Vx         [m/s]
%   u(6) = deltaPrev  [rad]
%
% Output:
%   deltaAFS          [rad]
%
% The vehicle prediction model is:
%
%   x(k+1) = Ad*x(k) + Bd*(deltaBase + deltaAFS)
%
% where:
%
%   x = [beta; r]

deltaBase = double(u(1));
beta      = double(u(2));
r         = double(u(3));
rRef      = double(u(4));
Vx        = double(u(5));
deltaPrev = double(u(6));

% Read common project parameters from the base workspace.
vehicle = evalin("base", "vehicle");
afs     = evalin("base", "afs");

VxModel = max(abs(Vx), afs.minSpeed);

persistent cachedKey
persistent Phi Gamma Qbar Rdbar E H qpOptions
persistent betaPerYaw deltaPerYaw

% This key is used to rebuild the MPC matrices only when the vehicle,
% speed, horizon, or weighting parameters change.
currentKey = [ ...
    VxModel;
    vehicle.m;
    vehicle.Iz;
    vehicle.a;
    vehicle.b;
    vehicle.Caf;
    vehicle.Car;
    afs.Ts;
    afs.Np;
    afs.Q(:);
    afs.R;
    afs.Rd];

rebuildMatrices = isempty(cachedKey) ...
    || numel(cachedKey) ~= numel(currentKey) ...
    || any(abs(cachedKey - currentKey) > 1e-12);

if rebuildMatrices
    m   = vehicle.m;
    Iz  = vehicle.Iz;
    a   = vehicle.a;
    b   = vehicle.b;
    Caf = vehicle.Caf;
    Car = vehicle.Car;

    %% Continuous-time 2-DOF bicycle model
    %
    % States:
    %   x = [beta; r]
    %
    % Input:
    %   delta_f = deltaBase + deltaAFS

    A = [ ...
        -(Caf + Car)/(m*VxModel), ...
        -(a*Caf - b*Car + m*VxModel^2)/(m*VxModel^2);

        -(a*Caf - b*Car)/Iz, ...
        -(a^2*Caf + b^2*Car)/(Iz*VxModel)];

    B = [ ...
        Caf/(m*VxModel);
        a*Caf/Iz];

    %% Steady-state cornering reference
    % At steady state:
    %
    %   0 = A*[beta_ref; r_ref] + B*delta_eq
    %
    % Rearranging the equations gives:
    %
    %   [A11  B1] [beta_ref] = -[A12] r_ref
    %   [A21  B2] [delta_eq]   [A22]
    %
    % Solve once per unit yaw rate. The resulting gains are then multiplied
    % by the current filtered yaw-rate reference.
    
    equilibriumMatrix = [ ...
        A(1,1), B(1);
        A(2,1), B(2)];
    
    equilibriumPerYaw = equilibriumMatrix \ ...
        (-[A(1,2); A(2,2)]);
    
    betaPerYaw  = equilibriumPerYaw(1);
    deltaPerYaw = equilibriumPerYaw(2);
    
    %% Exact zero-order-hold discretization

    nx = size(A,1);
    nu = size(B,2);

    augmentedMatrix = expm([ ...
        A, B;
        zeros(nu, nx + nu)] * afs.Ts);

    Ad = augmentedMatrix(1:nx, 1:nx);
    Bd = augmentedMatrix(1:nx, nx+1:nx+nu);

    %% MPC prediction matrices

    Np = afs.Np;

    Phi   = zeros(nx*Np, nx);
    Gamma = zeros(nx*Np, Np);

    for i = 1:Np
        Phi((i-1)*nx+1:i*nx, :) = Ad^i;

        for j = 1:i
            Gamma((i-1)*nx+1:i*nx, j) = Ad^(i-j)*Bd;
        end
    end

    Qbar  = kron(eye(Np), afs.Q);
    Rbar  = afs.R  * eye(Np);
    Rdbar = afs.Rd * eye(Np);

    %% Steering-difference matrix
    %
    % DeltaU =
    % [u(1)-u_previous;
    %  u(2)-u(1);
    %  ...
    %  u(Np)-u(Np-1)]

    E = eye(Np);

    for i = 2:Np
        E(i,i-1) = -1;
    end

    H = 2*(Gamma'*Qbar*Gamma + Rbar + E'*Rdbar*E);
    H = (H + H')/2;

    % Small regularization for numerical robustness.
    H = H + 1e-9*eye(Np);

    qpOptions = optimoptions( ...
        "quadprog", ...
        "Display", "off", ...
        "Algorithm", "interior-point-convex");

    cachedKey = currentKey;
end

Np = afs.Np;

%% Current state

x0 = [beta; r];

%% Prediction assumptions

% During the first implementation, current baseline steering is assumed
% constant over the prediction horizon.
deltaBasePrediction = deltaBase*ones(Np,1);

%% Physically consistent steady-state reference

% The desired sideslip is not forced to zero during steady cornering.
% It is calculated from the bicycle-model equilibrium corresponding to
% the filtered desired yaw rate.

betaRef = betaPerYaw*rRef;
deltaEq = deltaPerYaw*rRef; %#ok<NASGU>

Xref = repmat([betaRef; rRef], Np, 1);

%% Predicted-state offset

% X = Phi*x0 + Gamma*deltaAFS + Gamma*deltaBase
offset = Phi*x0 + Gamma*deltaBasePrediction - Xref;

%% Steering-rate cost contribution

eDifference = zeros(Np,1);
eDifference(1) = -deltaPrev;

f = 2*(Gamma'*Qbar*offset ...
    + E'*Rdbar*eDifference);

%% Steering-rate constraints

deltaStepMax = afs.deltaRateMax*afs.Ts;

% E*U + eDifference <= deltaStepMax
% E*U + eDifference >= -deltaStepMax

Aineq = [ ...
     E;
    -E];

bineq = [ ...
    deltaStepMax*ones(Np,1) - eDifference;
    deltaStepMax*ones(Np,1) + eDifference];

%% AFS angle and total-steering constraints

% AFS-only constraint:
%   -deltaMax <= deltaAFS <= deltaMax
%
% Total-steering constraint:
%   -totalDeltaMax <= deltaBase + deltaAFS <= totalDeltaMax

lowerScalar = max( ...
    -afs.deltaMax, ...
    -afs.totalDeltaMax - deltaBase);

upperScalar = min( ...
     afs.deltaMax, ...
     afs.totalDeltaMax - deltaBase);

if lowerScalar > upperScalar
    warning("solveAFSMPCOnline:InfeasibleSteeringBounds", ...
        "No feasible AFS steering range. Applying zero correction.");
    deltaAFS = 0;
    return
end

lb = lowerScalar*ones(Np,1);
ub = upperScalar*ones(Np,1);

%% Solve the quadratic program

[Uopt, ~, exitflag] = quadprog( ...
    H, f, ...
    Aineq, bineq, ...
    [], [], ...
    lb, ub, ...
    [], qpOptions);

if isempty(Uopt) || exitflag <= 0
    % Safe fallback: keep the previous AFS command while respecting
    % the current angle constraints.
    deltaAFS = min(max(deltaPrev, lowerScalar), upperScalar);
else
    % Receding-horizon implementation: apply only the first command.
    deltaAFS = Uopt(1);
end

if ~isfinite(deltaAFS)
    deltaAFS = 0;
end

end