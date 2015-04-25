function [stance_char] = stance_sim(X0, L0)
%STANCE_SIM Summary of this function goes here
%   Detailed explanation goes here
%
% By: Roberto Shu
% Last Updated: 4/25/2015
% ----------------------------------------------------------------------- %

% Read input
xtd = X0(1);
ytd = X0(2);
xdot0 = X0(3);
ydot0 = X0(4);
theta0 = X0(5);

% Initial stance variables
T_sp = 0.2;
tspan = linspace(0,T_sp,100);
Lb0 = L0;
Lb0dot = cos(theta0)*xdot0 + sin(theta0)*ydot0;             % Radial velocity [m/s]
theta0dot = (-sin(theta0)*xdot0+cos(theta0)*ydot0)/L0;       % Angular velocity [rad/s]

% Perform integration of dynamics
[T,X] = ode45(@stance_dynamics,tspan,[Lb0, theta0, Lb0dot, theta0dot]);

% Find when leg fully expands and lift-offs
[~,idx_lo] = min(abs(X(2:end,1)-L0));

% Extract results
T = T(1:idx_lo);               % Time vector [s]
Lb = X(1:idx_lo,1);            % Leg length [m]
theta = X(1:idx_lo,2);         % Leg angle to ground [rad]
Lbdot = X(1:idx_lo,3);         % Leg compression velocity [m/s]
thetadot = X(1:idx_lo,4);      % Angular velocity [rad/s]

% Calculate body position
for i = 1:length(T)
    x(i) = xtd + Lb(i)*cos(theta(i));
    y(i) = ytd + Lb(i)*sin(theta(i));
end

% Foot location
xf = xtd*ones(1,length(x));
yf = ytd*ones(1,length(y));

% Prepare results for next phase
Lbdot_lo = Lbdot(end);
theta_lo = theta(end);
thetadot_lo = thetadot(end);

% Convert angular and radial velocity to cartesian
xdot_lo = Lbdot_lo*cos(theta_lo)-L0*thetadot_lo*sin(theta_lo);
ydot_lo = Lbdot_lo*sin(theta_lo)+L0*thetadot_lo*cos(theta_lo);

% Initial position
x_lo = x(end);
y_lo = y(end);

% Prepare output
stance_char.x = x;                          % COM x-trajectory [m]
stance_char.y = y;                          % COM y-tracjetory [m]
stance_char.x_lo = x_lo;                    % COM x-position @ Lift-off [m/s]
stance_char.y_lo = y_lo;                    % COM y-position @ Lift-off [m/s]
stance_char.xdot_lo = xdot_lo;              % x-velocity @ Lift-off [m/s]
stance_char.ydot_lo = ydot_lo;              % y-velocity @ Lift-off [m/s]
stance_char.xf = xf;                        % Foot x-trajectory [m]
stance_char.yf = yf;                        % Foot y-trajectory [m]
stance_char.theta_lo = theta_lo;            % Lift-off angle [rad]

end

