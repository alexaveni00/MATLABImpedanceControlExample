function [zdot, T1, T2, lambda] = FullDynWithConstraint(z, p)
% FullDynWithInclinedGround Dynamics with inclined ground constraint
%  z = [th1; thdot1; th2; thdot2]

% Extract states
th1    = z(1);  thdot1 = z(2);
th2    = z(3);  thdot2 = z(4);
q      = [th1; th2];
qdot   = [thdot1; thdot2];

% === 1) Normalize coordinates w.r.t. ground inclination ===
alpha = getappdata(p.fig, 'ground_angle');
ca    = cos(alpha);
sa    = sin(alpha);
R     = [ca -sa; sa ca];

% Position and velocity of end-effector in base frame
pos = ForwardKin(p.l1, p.l2, th1, th2);
J   = JacobianEndeffector(p.l1, p.l2, th1, th2);
v   = J * qdot;

% Extract components
x0 = pos(1);  y0 = pos(2);
vx = v(1);    vy = v(2);

% Rotate manually to avoid dimension mismatch
px  =  ca*x0 - sa*y0;
py  =  sa*x0 + ca*y0;
vnx =  ca*vx - sa*vy;
vny =  sa*vx + ca*vy;

y_rot  = py;    % height in inclined frame
vd_rot = vny;   % velocity normal to ground

% === 2) Unconstrained dynamics and control torques ===
[zdot_free, T1, T2] = FullDyn(z, p);
% 1) penetrazione geometrica positiva
epsilonToActive = max(0, p.yinit - y_rot);

% 2) attivo se delta supera epsilon
activeConstraint = epsilonToActive > 1e-3;
if ~activeConstraint
    lambda = 0;  % no contact, no reaction force
    zdot = zdot_free;  % use free dynamics
    return;
end

    tau = [T1; T2];
    [M, C, G] = MassCoriolisGravity(th1, th2, thdot1, thdot2, ...
    p.m1, p.m2, p.l1, p.l2, p.d1, p.d2, p.I1, p.I2, p.g);

    % === 3) Viscoelastic contact in inclined frame ===
    % Effective mass approximated using vertical Jacobian (row 2)
    n = [ca; sa]; % normale inclinata
    % fallback normal component
    m_eff= (1 / (n'*(J * (M \ J'))*n));

    % Hunt–Crossley parameters
    [k_HC, c_HC] = computeGroundHC(p.E1, p.nu1, p.R1, p.E2, p.nu2, p.R2, p.e_restitution, m_eff);

    % Ground constraint parameters
    params.yinit         = p.yinit;
    params.max_penetration   = HertzPenetration(p.m1+p.m2, k_HC); % max penetration for active contact
    params.stiffness     = k_HC;
    params.n             = 1.5;
    params.damping       = c_HC;
    params.m_eff        = m_eff;  % effective mass for damping
    params.penetration = min(HertzPenetration(m_eff, k_HC), params.max_penetration);
    
  
    [lambda, ~] = GroundConstraint(vd_rot, params);
    F_incl  = [0; -lambda]; % force in inclined frame
    tau_constraint = J' * F_incl;
    b = (tau - C - G + tau_constraint);
    qddot = M \ b;
    %qddot = R' * qddot_base;
    zdot  = [thdot1; qddot(1); thdot2; qddot(2)];
end
