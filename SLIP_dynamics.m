% Simulation of Conservative SLIP model dynamics during flight phase
% by Roberto Shu

% Clear worskpace
clear all; close all; clc;

% Initialize constant 
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

%% Flight Phase - descent

% Do simulation
X0 = [x0, y0, xdot0, ydot0, theta_td];

[Leg] = flight_sim(h_apex, L0, X0, descent);
x_flight,y_flight,xf_flight,yf_flight,xdot_impact,ydot_impact
%% Stance Phase path

% Touch-down characteristics
xtd = xf_flight(end);
ytd = yf_flight(end);

% Initial parameters
T_sp = 0.2;
tspan = linspace(0,T_sp,30);
theta0 = theta_td;
Lb0 = L0;
Lb0dot = cos(theta0)*xdot_impact + sin(theta0)*ydot_impact;
theta0dot = (-sin(theta0)*xdot_impact+cos(theta0)*ydot_impact)/L0;

X0 = [Lb0 theta0 Lb0dot theta0dot];               % Vector of initial conditions

% Intergrate 
[T,X] = ode45(@stance_dynamics,tspan,X0);

% Extract results
Lb = X(:,1);
theta = X(:,2);
Lbdot = X(:,3);
thetadot = X(:,4);

% Calculate body position
for i = 1:length(T)
    xb_stance(i) = xtd + Lb(i)*cos(theta(i));
    yb_stance(i) = ytd + Lb(i)*sin(theta(i));
end

% Foot location
xf_stance = xtd*ones(1,length(xb_stance));
yf_stance = ytd*ones(1,length(yb_stance));

% Concatenate position vector
x = [x_flight,xb_stance];
y = [y_flight,yb_stance];
xf = [xf_flight,xf_stance];
yf = [yf_flight,yf_stance];

Lbdot_lo = Lbdot(end-5);
theta_lo = theta(end-5);
thetadot_lo = thetadot(end-5);

% Convert angular and radial velocity to cartesian
xdot_lo = Lbdot_lo*cos(theta_lo)-L0*thetadot_lo*sin(theta_lo)
ydot_lo = Lbdot_lo*sin(theta_lo)+L0*thetadot_lo*cos(theta_lo)

%% Flight Phase - ascent
x_lo = x(end);
y_lo = y(end);

% Flight time
t_fa = ydot_lo/g;

% CoM trajectory calculation
t = linspace(0,t_fa,20);
x_flight2 = x_lo + xdot_lo*t;
y_flight2 = y_lo + ydot_lo*t-g/2*t.^2;

% Calculate foot location during flight
xf_flight2 = x_flight2-L0*cos(theta_lo);
yf_flight2 = y_flight2-L0*sin(theta_lo);

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

figure
% Animation
for i = 1:length(x)
    plot(x,y,'g:');
    hold on;
    plot(x(i),y(i),'o','MarkerSize',10,'MarkerFaceColor','b')
    plot([x(i),xf(i)],[y(i),yf(i)])
    axis([0,0.5,0,1])
    xlabel('x-positon [m]')
    ylabel('y-positon [m]')
    pause(0.2);
    hold off
end

