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
% completed: true if the trajectory is completed
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [xt, yt, completed] = DefineTrajectory(t, x0, y0, T, terrainType)
    switch terrainType
        case 'soft'
            stepHeight = -0.125;
        case 'hard'
            stepHeight = 0;
        case 'step'
            stepHeight = 0.2;
    end
    % Define the radius of the semicircle
    r = 0.7;
    % Calculate the phase of the trajectory
    phase = mod(t, T) / T; % tempo normalizzato tra 0 e 1
    theta = pi * phase / 0.5;
    completed = false;
    if phase < 0.5
        xt = x0 + r * cos(theta);
        yt = y0 + r * sin(theta) + stepHeight;
        return;
    end

    if phase > 0.5
        xt = x0 + r * cos(0) - 2*r;
        yt = y0 + r * sin(0) + stepHeight;
        completed = true;
        return;
    end
end







function [xt, yt, completed] = DefineTwqfqw3frajectory(t, x0, y0, T)
    % Define the radius of the semicircle
    r = 0.7;
    % Define the height of the step
    stepHeight = 0.2;
    % Calculate the phase of the trajectory
    phase = mod(t, T) / T; % tempo normalizzato tra 0 e 1
    theta = pi * phase / 0.5;
    completed = false;
    
    if phase < 0.5
        % Movimento lungo il semicerchio
        xt = x0 + r * cos(theta);
        yt = y0 + r * sin(theta);
        return;
    end

    if phase >= 0.5
        % Movimento lineare verso il gradino
        xt = x0 + r * cos(0) - 2*r;
        yt = y0 + stepHeight; % Altezza del gradino
        completed = true;
        return;
    end
end