function [ineq_violations, eq_violations] = constraints( p, X0 )
%CONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here
% By Roberto Shu
% Last Updated: 4/25/2015
% ----------------------------------------------------------------------- %

%% Read input
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

% Do simulation
X = [x0 y0 xb_dot yb_dot theta_td];
[ COMtrajectory, Foottrajectory, stance_char] = SLIP_sim( Leg, X );

y = COMtrajectory.y;


Lb = stance_char.Lb;

%% Implement constraints subject to:
%   dynamics:
%   v_final: v_final = v_start

temp_ceq = [];
temp_c  = [];

%% Equality constraints
% s.t. v_final


%% Inequality constraints
idx = 0;
% for(i = 1:length(y))
%     idx = idx + 1;
%     temp_c(idx) = -y(i);
% end

% s.t. Lb >= Lf + Lfmin
Lbmin = min(Lb);
idx = idx + 1;
temp_c(idx) = Lf + Lkmin - Lbmin;

% s.t. 0 < theta_td < 180 
idx = idx + 1;
temp_c(idx) = theta_td - pi();
idx = idx + 1;
temp_c(idx) = pi()/2-theta_td;

% Set violations
eq_violations = temp_ceq';
ineq_violations = temp_c';
end

