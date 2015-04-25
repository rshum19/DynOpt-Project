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


X0 = [xb_dot, theta_td, y_apex];

% Do optimization
[x,fval,exitflag,output] = fmincon(@criterion,X0,[],[],[],[],[],[],@constraints,options);