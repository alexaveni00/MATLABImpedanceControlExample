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
v_ee = J * qdot;
v_ee_y = v_ee(2);

% Torques di controllo e dinamica non vincolata
[zdot_free, T1, T2] = FullDyn(z, p);
tau = [T1; T2];
[M, C, G] = MassCoriolisGravity(th1, th2, thdot1, thdot2, p.m1, p.m2, p.l1, p.l2, p.d1, p.d2, p.I1, p.I2, p.g);

% Calcola massa effettiva per il vincolo orizzontale
% Jn è la riga del Jacobiano relativa alla velocità verticale dell'end-effector
% Calcolo anche il ground_damping
Jn = J(2,:);    % 1×2
m_eff = 1 / ( Jn * (M \ Jn') );

[k_HC, c_HC] = computeGroundHC( ...
    p.E1, p.nu1, p.R1, ...
    p.E2, p.nu2, p.R2, ...
    p.e_restitution, m_eff); 
% Parametri terreno per la funzione GroundConstraint
params_terreno.yinit = p.yinit;
params_terreno.epsilon = 1e-3;
params_terreno.stiffness = k_HC;
params_terreno.n = 1.5;  % esponente per il modello Hunt-Crossley
params_terreno.damping = c_HC;

persistent lambda_max_persist contact_detected

if isempty(lambda_max_persist)
    lambda_max_persist = 0;
end
if isempty(contact_detected)
    contact_detected = false;
end

% Legge lo stato di contatto solo se è disponibile il riferimento alla figura
if isfield(p, 'fig') && ishandle(p.fig)
    contatto_attivo = getappdata(p.fig, 'contatto_attivo');
else
    contatto_attivo = true;  % se manca il campo, per sicurezza lo attiviamo
end

% Esegue il calcolo solo se abilitato
if ~contact_detected && contatto_attivo && y_ee <= p.yinit && v_ee_y < 0
    lambda_max_persist = MaxEndEffectorForce(z, p);
    contact_detected = true;
    fprintf('[INFO] Primo contatto rilevato: lambda_max = %.2f\n', lambda_max_persist);
end

params_terreno.lambda_max = lambda_max_persist;


% Vincolo attivo solo se p.enable_constraint == true
if isfield(p, 'enable_constraint') && p.enable_constraint
    [lambda, vincolo_attivo, info] = GroundConstraint(y_ee, v_ee_y, params_terreno);
else
    lambda = 0;
    vincolo_attivo = false;
end


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