% Simulation of Conservative SLIP model dynamics during flight phase
% by Roberto Shu

%% Clear worskpace
clear all; close all; clc;

%% Initialize constant 
g = 9.81;       % Acceleration due to gravity [m/s^2]
h_apex = 0.8;   % Apex height [m]
L0 = 0.5;       % Max lenght of leg [m]
descent = 0;
ascent = 1;

% Initialize initial conditions: starts at the apex
x0 = 0;
y0 = h_apex;
xdot0 = 0.5;                % COM x-velocity [m/s]
ydot0 = 0;                  % COM y-velocity [m/s], 0 becuase @ apex
theta_lo = pi()/3;          % Lift-off angle
theta_td = degtorad(95);    % Touch-down angle  
theta_td = degtorad(170);

%% Flight Phase - descent

% Do simulation
X0 = [x0, y0, xdot0, ydot0, theta_td];
[Leg_flight] = flight_sim(h_apex, L0, X0, descent);

x_flight = Leg_flight.x;
y_flight = Leg_flight.y;
xf_flight = Leg_flight.xf;
yf_flight = Leg_flight.yf;
xdot_impact = Leg_flight.xdot_end;
ydot_impact = Leg_flight.ydot_end;

figure
plot(x_flight,y_flight);

%% Stance Phase path

% Touch-down characteristics
xtd = xf_flight(end);
ytd = yf_flight(end);
xdot_td = xdot_impact;
ydot_td = ydot_impact;

% Simulate stance phase
X0 = [xtd, ytd, xdot_td, ydot_td, theta_td];
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

%% Result plots

figure
subplot(3,1,1)
plot(x_flight,y_flight)
subplot(3,1,2)
plot(xb_stance,yb_stance)
subplot(3,1,3)
plot(x_flight2,y_flight2)

% Animation
figure
for i = 1:length(x)
    plot(x,y,'g:');
    hold on;
    plot(x(i),y(i),'o','MarkerSize',10,'MarkerFaceColor','b')
    plot([x(i),xf(i)],[y(i),yf(i)])
    axis([0,1,0,1])
    xlabel('x-positon [m]')
    ylabel('y-positon [m]')
    pause(0.2);
    hold off
end

