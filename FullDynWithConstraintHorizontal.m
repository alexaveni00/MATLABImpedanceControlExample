function [zdot, T1, T2, lambda] = FullDynWithInclinedGround(z, p)
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
tau = [T1; T2];
[M, C, G] = MassCoriolisGravity(th1, th2, thdot1, thdot2, ...
    p.m1, p.m2, p.l1, p.l2, p.d1, p.d2, p.I1, p.I2, p.g);

% === 3) Viscoelastic contact in inclined frame ===
% Effective mass approximated using vertical Jacobian (row 2)
Jn   = J(2,:);                % fallback normal component
m_eff= 1 / (Jn * (M \ Jn'));

% Hunt–Crossley parameters
[k_HC, c_HC] = computeGroundHC(p.E1, p.nu1, p.R1, p.E2, p.nu2, p.R2, p.e_restitution, m_eff);

% Ground constraint parameters
params.yinit      = p.yinit;
params.epsilon    = 1e-3;
params.stiffness  = k_HC;
params.n          = 1.5;
params.damping    = c_HC;
params.lambda_max = MaxEndEffectorForce(z, p);

% Compute penetration and reaction
lambda = 0;
zdot   = zdot_free;
active = false; info = struct();
if isfield(p,'enable_constraint') && p.enable_constraint
    [lambda, active, info] = GroundConstraint(y_rot, vd_rot, params);
end

% === 4) Apply constraint force if active ===
if active
    % Reaction force (direction perpendicular to original horizontal, approximate)
    F_base = [0; -lambda];
    tau_constraint = J' * F_base;
    qddot = M \ (tau - C - G + tau_constraint);
    zdot  = [thdot1; qddot(1); thdot2; qddot(2)];
end
