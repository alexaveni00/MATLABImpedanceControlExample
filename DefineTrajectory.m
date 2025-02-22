%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define the trajectory of the robot end effector
%
% Inputs:
% t: time
% x0: initial x position
% y0: initial y position
% T: period of the trajectory
%
% Outputs:  
% xt: x position of the end effector
% yt: y position of the end effector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [xt, yt] = defineTrajectory(t, x0, y0, T)
    % Define the radius of the semicircle
    r = 0.7;
    % Calculate the phase of the trajectory
    phase = mod(t, T) / T; % tempo normalizzato tra 0 e 1
    theta = pi * phase / 0.5;

    if phase < 0.5
        xt = x0 + r * cos(theta);
        yt = y0 + r * sin(theta);
        return;
    end

    if phase > 0.5
        xt = x0 + r * cos(0) - 2*r;
        yt = y0 + r * sin(0);
        return;
    end
end