function [lambda, activeConstraint, info] = GroundConstraint(y_ee, v_ee_y, params)
%GROUND CONSTRAINT - Hunt–Crossley viscoelastico
%   F = k*delta^n + c*delta^n * v_ee_y

    % 1) penetrazione geometrica positiva
    delta = max(0, params.yinit - y_ee);

    % 2) attivo se delta supera epsilon
    activeConstraint = delta > params.epsilon;

    % init info
    info = struct('penetration', delta, ...
                  'springForce', 0, ...
                  'dampingForce', 0);

    if ~activeConstraint
        lambda = 0;
        return
    end

    % 3) calcolo forc e damping non-lineari
    n = params.n;  % es. 1.5 per Hertz
    F_spring  = params.stiffness * delta^n;
    F_damp    = params.damping   * delta^n * max(0, -v_ee_y);

    % 4) salva info
    info.springForce  = F_spring;
    info.dampingForce = F_damp;

    % 5) reazione totale saturata
    lambda = F_spring + F_damp;
    lambda = min(lambda, params.lambda_max);
end
