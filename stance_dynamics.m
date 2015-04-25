function [ dX ] = stance_dynamics( t,X )
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

% X = [Lb theta Lb_dot theta_dot];

% Initialize constant variables:
m = 7;                  % mass [kg]
L0 = 0.5;               % Length of leg during flight[m]
Lf = 0.3;               % Length of the unsprung part of the leg [m]
Lk0 = 0.2;           % Length of the uncompressed spring [m]
k = 2800;               % Spring constant [N/m]
g = 9.81;               % gravity [m/s^2]
%Lb = L0;

Lb = X(1);
theta = X(2);
Lb_dot = X(3);
theta_dot = X(4);

dX = zeros(4,1);        % initialize column vector

dX(1) = X(3);
dX(2) = X(4);
dX(3) = -k/m*(Lb-Lf-Lk0) - g*sin(theta)-Lb*theta_dot^2;
dX(4) = (-2*Lb_dot*theta_dot-g*cos(theta))/Lb;

end

