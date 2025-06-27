function [lambda, vincolo_attivo] = GroundConstraint(y_ee, v_ee_y, params)
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


phi = y_ee - params.yinit;
vincolo_attivo = (phi < params.epsilon);

if vincolo_attivo
    % Modello molla-smorzatore (solo se penetra)
    lambda = params.stiffness * max(0, -phi) - params.damping * min(0, v_ee_y);
    lambda = min(lambda, params.lambda_max);
    lambda = max(lambda, 0); % nessuna trazione
else
    lambda = 0;
end
end
