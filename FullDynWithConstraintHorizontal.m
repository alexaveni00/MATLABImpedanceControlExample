function [zdot, T1, T2, lambda] = FullDynWithConstraintHorizontal(t, z, p)
% Dinamica con vincolo orizzontale y = p.yinit (vincolo attivo solo se y_ee < p.yinit)
% z = [th1; thdot1; th2; thdot2]

th1 = z(1); thdot1 = z(2);
th2 = z(3); thdot2 = z(4);
q = [th1; th2];
qdot = [thdot1; thdot2];

% Cinematica diretta e Jacobiano
pos = ForwardKin(p.l1, p.l2, th1, th2);
x_ee = pos(1); y_ee = pos(2);
J = JacobianEndeffector(p.l1, p.l2, th1, th2);

% Calcola velocità verticale end-effector
v_ee = J * [thdot1; thdot2];
v_ee_y = v_ee(2);

% Parametri terreno per la funzione GroundConstraint
params_terreno.yinit = p.yinit;
params_terreno.epsilon = 1e-3;
params_terreno.stiffness = p.lambda_max; % se vuoi terreno rigido, usa lambda_max come stiffness
params_terreno.damping = p.ground_damping;
params_terreno.lambda_max = p.lambda_max;

% Calcola reazione vincolare tramite funzione terreno
[lambda, vincolo_attivo] = GroundConstraint(y_ee, v_ee_y, params_terreno);

% Torques di controllo e dinamica non vincolata
[zdot_free, T1, T2] = FullDyn(t, z, p);
tau = [T1; T2];
[M, C, G] = MassCoriolisGravity(th1, th2, thdot1, thdot2, p.m1, p.m2, p.l1, p.l2, p.d1, p.d2, p.I1, p.I2, p.g);

% Applica la forza di reazione vincolare come forza esterna se il vincolo è attivo
if vincolo_attivo
    % Forza esterna verticale sull'end-effector
    F_ext = [0; -lambda];
    % Torques equivalenti alle articolazioni
    tau_constraint = J' * F_ext;
    % Dinamica con forza di reazione
    % Gestione singolarità/NaN per M
    if rcond(M) < 1e-8 || any(isnan(M(:))) || any(isinf(M(:)))
        qddot = [0;0];
    else
        qddot = M \ (tau - C - G + tau_constraint);
    end
    zdot = [thdot1; qddot(1); thdot2; qddot(2)];
else
    % Dinamica libera
    zdot = zdot_free;
    lambda = 0; % Assicurati che lambda sia zero quando il vincolo non è attivo
end
end
