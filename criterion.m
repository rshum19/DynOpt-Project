function [ score ] = criterion( p, X0 )
%CRITERION Summary of this function goes here
%   Detailed explanation goes here
% Inputs:
%   - p:            parameters to be optimized
%
% Outputs
%   - score:        cost function value
% By: Roberto Shu
% Last Updated: 4/25/2015
% ----------------------------------------------------------------------- %

% Simulate step
xb0_dot = X0(1);
theta0_td = X0(2);
h_apex0 = X0(3);

xb_dot = p(1);
yb_dot = 0;
theta_td = p(2);
h_apex = p(3);


Lkmin = 0.02;   % Min spring length [m]
Lf = 0.3;
L0 = 0.5;

x0 = 0;
y0 = h_apex0;

Leg.apex0 = h_apex0;
Leg.L0 = L0;

X = [x0 y0 xb_dot yb_dot theta_td];
[ COMtrajectory, Foottrajectory, stance_char, Leg_flight2] = SLIP_sim( Leg, X );

% Results
h_apex_start = h_apex0;
h_apex_end = COMtrajectory.y(end);
xdot_apex_start = xb0_dot;
xdot_apex_end = Leg_flight2.xdot_end;
thetatd_dot = stance_char.thetadot(1);


% Evaluate cost function

    score = thetatd_dot + (h_apex_start-h_apex_end)^2 + (xdot_apex_start-xdot_apex_end)^2;

end

