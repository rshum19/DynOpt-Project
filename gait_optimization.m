% Course:   16-745 Dynamic Optimization
% Purpose:  Optimization of a one-legged hopping gait using a conservative 
%           SLIP model
% Filename: gait_optimization.m
% Author:   Roberto Shu 
% ----------------------------------------------------------------------- %

%% ----------------------- Clear workspace --------------------------
clear all; close all; clc;

%% ----------------------- Optimization ------------------------------

% Optimziation options
options = optimoptions('fmincon','Algorithm','interior-point','MaxFunEvals',10e5,'TolFun',1e-5,'TolX', 1e-6,'Display','iter','UseParallel','never');

% Initial conditions
xb_dot = 0.5;
yb_dot = 0;
theta_td = degtorad(95);
h_apex0 = 0.8;
L0 = 0.5;

X0 = [xb_dot, theta_td, h_apex0];

% Do optimization
[x,fval,exitflag,output] = fmincon(@(p)criterion(p,X0),X0,[],[],[],[],[],[],@(p)constraints(p,X0),options);


Leg.apex0 = h_apex0;
Leg.L0 = L0;
X = [0 0.8 x(1) 0 x(2)];
[ COMtrajectory, Foottrajectory, stance_char] = SLIP_sim( Leg, X );

% Animation
figure
for i = 1:length(COMtrajectory.x)
    plot(COMtrajectory.x,COMtrajectory.y,'g:');
    hold on;
    plot(COMtrajectory.x(i),COMtrajectory.y(i),'o','MarkerSize',10,'MarkerFaceColor','b')
    plot([COMtrajectory.x(i),Foottrajectory.x(i)],[COMtrajectory.y(i),Foottrajectory.y(i)])
    axis([0,1,0,1])
    xlabel('x-positon [m]')
    ylabel('y-positon [m]')
    pause(0.2);
    hold off
end