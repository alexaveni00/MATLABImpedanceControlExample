%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Two link robot arm with control to track a point that the user clicks.
%
% Files:
% MAIN - Execute this file; parameters here.
%
% Plotter -- Handles all integration and graphics (symplectic Euler with dt cap)
%
% FullDynWithConstraintHorizontal -- Dynamics + constraints
% GroundConstraint -- Smoothed ground contact model
% SemicircleTrajectory, computeKpKd, deriverRelativeAngles as before.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; close all;
clear all;

rederive = false;
%%%%%%%% System Parameters %%%%%%%%

% Initial conditions:
p.init = [-2*pi/3; 0; -pi/2; 0];

% Physical constants
p.g = 9.81;
p.m1 = 0.5; p.m2 = 0.5;
p.l1 = 1;   p.l2 = 1;
p.d1 = p.l1/2; p.d2 = p.l2/2;
p.I1 = 1/12*p.m1*p.l1^2;
p.I2 = 1/12*p.m2*p.l2^2;

% Forward kinematics to get initial end-effector pos
endZ = ForwardKin(p.l1,p.l2,p.init(1),p.init(3));
x0 = endZ(1);  y0 = endZ(2);
p.Fx = 0;      p.Fy = 0;

%%%%%%%% Control Parameters %%%%%%%%
J0 = JacobianEndeffector(p.l1,p.l2,p.init(1),p.init(3));
qdot0 = [p.init(2); p.init(4)];
v_ee0 = J0*qdot0; vel_ee0_mod = norm(v_ee0);
[p.Kp, p.Kd] = computeKpKd(vel_ee0_mod);

% Single target at initial position
p.xtarget = x0;
p.ytarget = y0;
p.yinit   = y0;

% Parameters per gestione contatto e integrazione
p.ground_stiffness = 1000; % stiffness del terreno
p.ground_damping  = 50;     % damping terreno
p.penetration_max = 0.05;   % penetrazione massima ammessa
p.dt_max          = 0.01;   % passo dt massimo nel loop
p.enable_constraint = false; % Abilita vincolo orizzontale
if rederive
    deriverRelativeAngles;
    disp('Equazioni rederive.');
end

% Parametri traiettoria semicirconferenza + estensione
p.theta_start = 0;
p.theta_end   = pi + pi/8;

% Avvia animazione & integrazione
Plotter(p);
