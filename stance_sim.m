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
hit_ground = 0;
Lf = 0.3;
Lkmin = 0.02;
Lkmax = 0.18;
Lk0 = Lkmin + Lkmax;
T_sp = 1;
tspan = linspace(0,T_sp,30);
Lb0 = L0;
Lb0dot = cos(theta0)*xdot0 + sin(theta0)*ydot0;             % Radial velocity [m/s]
theta0dot = (-sin(theta0)*xdot0+cos(theta0)*ydot0)/L0;       % Angular velocity [rad/s]

% Perform integration of dynamics
options = odeset('RelTol',1e-13,'AbsTol',1e-14);
[T,X] = ode113(@stance_dynamics,[0 T_sp],[Lb0, theta0, Lb0dot, theta0dot]);%,options);

% Extract results
Lb = X(:,1);            % Leg length [m]
theta = X(:,2);         % Leg angle to ground [rad]
Lbdot = X(:,3);         % Leg compression velocity [m/s]
thetadot = X(:,4);      % Angular velocity [rad/s]

% Calculate body position
for i = 1:length(T)
    x(i) = xtd + Lb(i)*cos(theta(i));
    y(i) = ytd + Lb(i)*sin(theta(i));
end

% Find if COM falls to ground
hit_idx = find(y <= 0 );
if (length(hit_idx) > 0)
    hit_ground = 1;
    y = [y(1:hit_idx(1)-1),0];
    x = x(1:hit_idx(1));
    Lb = Lb(1:hit_idx(1));
    Lbdot  = Lbdot(1:hit_idx(1));
    theta = theta(1:hit_idx(1));
    thetadot = thetadot(1:hit_idx(1));
    T = T(1:hit_idx(1));
end

% Check if lift-off happens
liftoff = 0;
Lbidx = find( Lb(2:end) >= (Lf+Lk0));% + Lkmax));
if (Lbidx > 0)
    liftoff = 1;
    liftoff_idx = Lbidx(1)+1;
    
    x = x(1:liftoff_idx);
    y = y(1:liftoff_idx);
    Lb = Lb(1:liftoff_idx);
    Lb = Lb(1:liftoff_idx);
    Lbdot  = Lbdot(1:liftoff_idx);
    theta = theta(1:liftoff_idx);
    thetadot = thetadot(1:liftoff_idx);
    T = T(1:liftoff_idx);
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
if(hit_ground & ~liftoff)
    xdot_lo(end) = 0;
    ydot_lo(end) = 0;
end

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
stance_char.Lb = Lb;                        % Spring compression trajectory
stance_char.period = T(end);                % Stance phase time period
stance_char.thetadot = thetadot;
stance_char.hit_ground = hit_ground;        % Boolean identifiyin if (1) hit ground (0) did not hit ground
stance_char.liftoff = liftoff;
end

