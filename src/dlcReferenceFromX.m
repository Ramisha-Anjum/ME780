function [Xref, Yref, psiRef, kappaRef, VxRef, muOut] = ...
    dlcReferenceFromX( ...
    X, Vx, mu, laneWidth, xStart, transitionLength, holdLength)
%dlcReferenceFromX Generate a smooth double-lane-change reference.
%
% Inputs
%   X                actual longitudinal vehicle position [m]
%   Vx               reference longitudinal speed [m/s]
%   mu               road-friction coefficient
%   laneWidth        lateral lane displacement [m]
%   xStart           longitudinal start of first transition [m]
%   transitionLength longitudinal length of each transition [m]
%   holdLength       distance travelled in adjacent lane [m]
%
% Outputs
%   Xref       reference longitudinal position [m]
%   Yref       reference lateral position [m]
%   psiRef     reference path heading [rad]
%   kappaRef   reference path curvature [1/m]
%   VxRef      reference longitudinal speed [m/s]
%   muOut      road-friction coefficient
%
% The transition uses a fifth-order smoothstep:
%
%   h(q) = 10q^3 - 15q^4 + 6q^5
%
% It provides zero first and second derivatives at both ends, avoiding
% discontinuous heading and curvature commands.

Xref = X;
VxRef = Vx;
muOut = mu;

x1 = xStart;
x2 = x1 + transitionLength;
x3 = x2 + holdLength;
x4 = x3 + transitionLength;

Yref = 0;
dYdX = 0;
d2YdX2 = 0;

if X < x1
    % Initial straight lane.
    Yref = 0;

elseif X <= x2
    % Move from original lane to adjacent lane.
    q = (X - x1)/transitionLength;

    [h, dh, d2h] = quinticSmoothstep(q);

    Yref = laneWidth*h;
    dYdX = laneWidth/transitionLength*dh;
    d2YdX2 = laneWidth/transitionLength^2*d2h;

elseif X < x3
    % Stay in adjacent lane.
    Yref = laneWidth;

elseif X <= x4
    % Return from adjacent lane to original lane.
    q = (X - x3)/transitionLength;

    [h, dh, d2h] = quinticSmoothstep(q);

    Yref = laneWidth*(1 - h);
    dYdX = -laneWidth/transitionLength*dh;
    d2YdX2 = -laneWidth/transitionLength^2*d2h;

else
    % Final straight lane.
    Yref = 0;
end

% Path heading.
psiRef = atan(dYdX);

% Exact curvature of y = f(x).
kappaRef = d2YdX2/(1 + dYdX^2)^(3/2);

end


function [h, dh, d2h] = quinticSmoothstep(q)
%quinticSmoothstep Fifth-order transition and its derivatives.

% Protect against small numerical excursions outside [0,1].
q = min(max(q, 0), 1);

h = 10*q^3 - 15*q^4 + 6*q^5;

dh = 30*q^2 - 60*q^3 + 30*q^4;

d2h = 60*q - 180*q^2 + 120*q^3;

end