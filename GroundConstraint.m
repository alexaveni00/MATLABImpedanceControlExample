function [lambda, activeConstraint, info] = GroundConstraint(y_ee, v_ee_y, params)
%GROUND CONSTRAINT - Modello terreno generico (rigido + smorzamento corretto)
%   y_ee: posizione verticale end-effector
%   v_ee_y: velocità verticale end-effector
%   params: struct con i parametri del terreno
%       .yinit: quota terreno
%       .stiffness: rigidità (N/m)
%       .damping: smorzamento (N/(m/s))
%       .lambda_max: forza massima
%       .epsilon: tolleranza attivazione vincolo
%
%   Output:
%       lambda: forza di reazione vincolare (>=0)
%       activeConstraint: booleano
%       info: struct con campi .penetration, .dampingForce, .springForce

    % Distanza rispetto al livello del terreno
    phi = y_ee - params.yinit;
    % Attivo solo se sotto la soglia di epsilon
    activeConstraint = (phi < params.epsilon);

    % Inizializzo info
    info = struct('penetration', 0, 'springForce', 0, 'dampingForce', 0);

    if activeConstraint
        % Penetrazione positiva
        penetration   = max(0, -phi);
        % Forza molla
        springForce   = params.stiffness * penetration;
        % Forza di damping (attiva solo se v_ee_y < 0)
        dampingForce  = params.damping  * max(0, -v_ee_y);

        % Salvo info utili
        info.penetration = penetration;
        info.springForce = springForce;
        info.dampingForce= dampingForce;

        % Sommo le forze e poi saturo
        lambda = springForce + dampingForce;
        lambda = min(lambda, params.lambda_max);
        lambda = max(lambda, 0);
    else
        lambda = 0;
    end
end
