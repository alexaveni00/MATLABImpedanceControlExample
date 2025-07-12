function [lambda, info] = GroundConstraint(v_ee_y, params)
% GROUND CONSTRAINT - Hunt–Crossley viscoelastico)
%GROUND CONSTRAINT - Hunt–Crossley viscoelastico
%   F = k*delta^n + c*delta^n * v_ee_y

    % init info
    info = struct('springForce', 0, ...
                  'dampingForce', 0);

    % 3) calcolo forc e damping non-lineari
    n = params.n;  % es. 1.5 per Hertz
    
    F_spring  = params.stiffness * params.penetration^n;
    F_damp    = params.damping   * params.penetration^n * max(0, -v_ee_y);

    % 4) salva info
    info.springForce  = F_spring;
    info.dampingForce = F_damp;

    % 5) reazione totale saturata
    lambda = F_spring + F_damp;
end
