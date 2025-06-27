function [lambda, vincolo_attivo, info] = GroundConstraint(y_ee, v_ee_y, params)
%GROUND CONSTRAINT - Modello terreno generico (rigido + smorzamento)
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
%       vincolo_attivo: booleano
%       info: struct opzionale (penetrazione, ecc)

if ~isfield(params, 'epsilon')
    params.epsilon = 1e-3;
end
if ~isfield(params, 'stiffness')
    params.stiffness = 1e5; % default: terreno rigido
end
if ~isfield(params, 'damping')
    params.damping = 0;
end
if ~isfield(params, 'lambda_max')
    params.lambda_max = 1e6;
end

phi = y_ee - params.yinit;
vincolo_attivo = (phi < params.epsilon);

if vincolo_attivo
    penetrazione = -phi; % quanto l'end-effector è sotto la quota
    % Modello molla-smorzatore (solo se penetra)
    lambda = params.stiffness * max(0, -phi) - params.damping * min(0, v_ee_y);
    lambda = min(lambda, params.lambda_max);
    lambda = max(lambda, 0); % nessuna trazione
else
    lambda = 0;
    penetrazione = 0;
end

if nargout > 2
    info.penetrazione = penetrazione;
    info.phi = phi;
end
end
