function [ COMtrajectory, Foottrajectory, stance_char, Leg_flight2] = SLIP_sim( Leg, X0 )
%SLIP_SIM Summary of this function goes here
%   Detailed explanation goes here
% By Roberto Shu
% Last Updated: 4/25/2015
% ----------------------------------------------------------------------- %

%% Initialize constant 
g = 9.81;               % Acceleration due to gravity [m/s^2]
descent = 0;
ascent = 1;

% Read input
h_apex = Leg.apex0;     % Apex height [m]
L0 = Leg.L0;            % Max lenght of leg [m]

% Initial conditions
x0 = X0(1);
y0 = X0(2);
xdot0 = X0(3);           % COM x-velocity [m/s]
ydot0 = X0(4);           % COM y-velocity [m/s]
theta_td0 = X0(5);       % Touch-down angle  

%% Flight Phase - descent

% Do simulation
X0 = [x0, y0, xdot0, ydot0, theta_td0];
[Leg_flight] = flight_sim(h_apex, L0, X0, descent);

x_flight = Leg_flight.x;
y_flight = Leg_flight.y;
xf_flight = Leg_flight.xf;
yf_flight = Leg_flight.yf;
xdot_impact = Leg_flight.xdot_end;
ydot_impact = Leg_flight.ydot_end;

%% Stance Phase path

% Touch-down characteristics
xtd = xf_flight(end);
ytd = yf_flight(end);
xdot_td = xdot_impact;
ydot_td = ydot_impact;

% Simulate stance phase
X0 = [xtd, ytd, xdot_td, ydot_td, theta_td0];
[stance_char] = stance_sim(X0, L0);

xb_stance = stance_char.x;
yb_stance = stance_char.y;
x_lo = stance_char.x_lo;
y_lo = stance_char.y_lo; 
xdot_lo = stance_char.xdot_lo; 
ydot_lo = stance_char.ydot_lo; 
xf_stance = stance_char.xf;
yf_stance = stance_char.yf;
theta_lo = stance_char.theta_lo;

%% Flight Phase - ascent

% Simulate ascent flight phase
X0 = [x_lo, y_lo, xdot_lo, ydot_lo, theta_lo];
[Leg_flight2] = flight_sim(h_apex, L0, X0, ascent);

x_flight2 = Leg_flight2.x;
y_flight2 = Leg_flight2.y;
xf_flight2 = Leg_flight2.xf;
yf_flight2 = Leg_flight2.yf;
xdot_impact2 = Leg_flight2.xdot_end;
ydot_impact2 = Leg_flight2.ydot_end;

% Concatenate position vector
x = [x_flight,xb_stance,x_flight2];
y = [y_flight,yb_stance,y_flight2];
xf = [xf_flight,xf_stance,xf_flight2];
yf = [yf_flight,yf_stance,yf_flight2];

%% Prepare results
COMtrajectory.x = x;
COMtrajectory.y = y;
Foottrajectory.x = xf;
Foottrajectory.y = yf;
end

