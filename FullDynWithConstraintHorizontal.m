function [zdot, T1, T2, lambda] = FullDynWithConstraintHorizontal(z, p)
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
params_terreno.stiffness = p.ground_stiffness;
params_terreno.damping = p.ground_damping;
params_terreno.lambda_max = MaxEndEffectorForce(z,p);

% Vincolo attivo solo se p.enable_constraint == true
if isfield(p, 'enable_constraint') && p.enable_constraint
    [lambda, vincolo_attivo, info] = GroundConstraint(y_ee, v_ee_y, params_terreno);
else
    lambda = 0;
    vincolo_attivo = false;
end

% Torques di controllo e dinamica non vincolata
[zdot_free, T1, T2] = FullDyn(z, p);
tau = [T1; T2];
[M, C, G] = MassCoriolisGravity(th1, th2, thdot1, thdot2, p.m1, p.m2, p.l1, p.l2, p.d1, p.d2, p.I1, p.I2, p.g);

if vincolo_attivo
    F_ext = [0; -min(lambda, params_terreno.lambda_max)];
    tau_constraint = J' * F_ext;
    if rcond(M) < 1e-8 || any(isnan(M(:))) || any(isinf(M(:)))
        qddot = [0;0];
    else
        qddot = M \ (tau - C - G + tau_constraint);
    end
    zdot = [thdot1; qddot(1); thdot2; qddot(2)];
else
    zdot = zdot_free;
    lambda = 0;
end
end
