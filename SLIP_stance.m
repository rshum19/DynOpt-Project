% Simulation of SLIP stance dynamics
% by Roberto Shu

% Clear workspace
clear all; close all; clc;

% Need to use ode45
T_sp = 1;                                           % Stance phase period [s]
L0 = 0.5;
xdot = 0.6;
ydot = -3;
theta0 = degtorad(95);
Lb0 = L0;
m = 7;  
k = 2800;
%T_sp = pi()*sqrt(m/k);
T_sp = 0.2;

% Calculate touch-down location
xtd = 0.4157;
ytd = 0;

% Initial parameters
Lb0dot = cos(theta0)*xdot + sin(theta0)*ydot;
theta0dot = (-sin(theta0)*xdot+cos(theta0)*ydot)/L0;

X0 = [Lb0 theta0 Lb0dot theta0dot];               % Vector of initial conditions

% Intergrate 
[T,X] = ode45(@stance_dynamics,[0,T_sp],X0);

% Extract results
Lb = X(:,1);
theta = X(:,2);
Lbdot = X(:,3);
thetadot = radtodeg(X(:,4));

% Calculate body position
for i = 1:length(T)
    xb(i) = xtd + Lb(i)*cos(theta(i));
    yb(i) = ytd + Lb(i)*sin(theta(i));
end
% Foot location
xf = xtd*ones(1,length(xb));
yf = ytd*ones(1,length(yb));

% Animation
figure
for i = 1:length(xb)
    plot(xb,yb,'g:')
    hold on;
    plot(xb(i),yb(i),'o','MarkerSize',10,'MarkerFaceColor','b')
    plot([xb(i),xf(i)],[yb(i),yf(i)])
    title('X-Y COM position')
    xlabel('x-positon [m]')
    ylabel('y-positon [m]')
    pause(0.2);
    hold off;
end

figure
plot(T,yb)
title('Y COM position')

figure
plot(T,X(:,1))
title('Leg length')

figure
plot(T,radtodeg(X(:,2)))
title('Angle position')

figure
plot(T,X(:,3))
title('Leg compresion velocity')

figure
plot(T,X(:,4))
title('Angular velocity')