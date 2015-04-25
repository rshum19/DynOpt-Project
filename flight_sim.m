function [Leg] = flight_sim(h_apex, L0, X0, phase)
%FLIGHT_SIM simulates the trajectory of the one-legged hopper during the
%flight phase
%
% Inputs:
%   - h_apex:       Initial apex height
%   - L0:           Full extended leg
%   - X0:           vector of initial conditions [x0, y0, xdot0, ydot0, theta]
%   - phase:        phase = 1 or 0 to indicate if ascent or descent
%                   respectively
% Outputs:
%   - Leg:          Structure component that contains the cartesian
%                   position of the CoM and foot, it also has the 
%                   vecolity at apex/impact 
% By: Roberto Shu
% Last Updated: 4/25/2015
% ----------------------------------------------------------------------- %

% Initialize constant variables
descent = 0;
ascent = 1;
g = 9.81;                   % Acceleration due to gravity [m/s^2]

% Read input
x0 = X0(1);                 % CoM x-position [m]
y0 = X0(2);                 % CoM y-positon [m]
xdot0 = X0(3);              % CoM x-velocity [m/s]
ydot0 = X0(4);              % CoM y-velocity [m/s]                  
theta = X0(5);           % Touch-down/Lift-off angle [rad]


if (phase == descent)
    % Calculate y-position of COM at foot impact 
    ytd = L0*sin(theta);
    
    t_flight = sqrt(2*(y0-ytd)/g);  % Flight time [s]
    
elseif(phase == ascent)
    
    t_flight = ydot0/g;             % Flight time [s]
end

% Define flight phase dynamics
xddot = 0;
yddot = -g;

% Calculate flight phase characteristics:
range = x0*t_flight+xddot*t_flight;     % Range [m]
xdot_end = xdot0;                       %  X-velocity @ impact/apex
ydot_end = ydot0 + yddot*t_flight;      % Y-velocity @ impact/apex

% CoM trajectory calcualtion
t = linspace(0,t_flight,20);
x = x0 + xdot0*t;
y = y0 + ydot0*t-g/2*t.^2;

% Calculate foot location during flight
xf = x-L0*cos(theta);
yf = y-L0*sin(theta);
    
% Prepare output
Leg.x = x;
Leg.y = y;
Leg.xf = xf;
Leg.yf = yf;
Leg.xdot_end = xdot_end;
Leg.ydot_end = ydot_end;

end

