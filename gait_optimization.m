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
options = optimoptions('fmincon','Algorithm','interior-point','MaxFunEvals',10e5,'TolFun',1e-5,'TolX', 1e-3,'Display','iter','UseParallel','always');

% Initial conditions
xb_dot = 0.5;
yb_dot = 0;
theta_td = degtorad(95);
h_apex0 = 0.8;

X0 = [xb_dot, theta_td, h_apex0];

% Do optimization
[x,fval,exitflag,output] = fmincon(@(p)criterion(p,X0),X0,[],[],[],[],[],[],@(p)constraints(p,X0),options);