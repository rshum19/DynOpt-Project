function [ score ] = criterion( p )
%CRITERION Summary of this function goes here
%   Detailed explanation goes here
% Inputs:
%   - p:            parameters to be optimized
%
% Outputs
%   - score:        cost function value
% By: Roberto Shu
% Last Updated: 4/25/2015
% ----------------------------------------------------------------------- %

% Simulate step

% Evaluate cost function

    score = thetatd_dot + (h_apex_start-h_apex_end)^2 + (xdot_apex_start-xdot_apex_end)^2;

end

