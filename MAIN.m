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

rederive = false;
%%%%%%%% System Parameters %%%%%%%%

% Initial conditions:
p.init = [-2*pi/3; 0; -pi/2; 0];

% Physical constants
p.g = 9.81;
p.m1 = 2; p.m2 = 2;
p.l1 = 1;   p.l2 = 1;
p.d1 = p.l1/2; p.d2 = p.l2/2;
p.I1 = 1/12*p.m1*p.l1^2;
p.I2 = 1/12*p.m2*p.l2^2;

% Forward kinematics to get initial end-effector pos
endZ = ForwardKin(p.l1,p.l2,p.init(1),p.init(3));
x0 = endZ(1);  y0 = endZ(2);

%%%%%%%% Control Parameters %%%%%%%%
[p.Kp, p.Kd] = computeKpKd(p.init(4)); % calcola Kp e Kd in base alla velocità angolare iniziale del giunto del "ginocchio" (thdot2)

% Single target at initial position
p.xtarget = x0;
p.ytarget = y0;
p.yinit   = y0;
p.ground_angle = 0; % angolo iniziale del terreno (orizzontale)
% Parameters per gestione contatto e integrazione
if rederive
    deriverRelativeAngles;
    disp('Equazioni rederive.');
end

% Avvia animazione & integrazione
% === Inizializza file CSV per logging ===
csv_filename = 'log_dati_simulazione.csv';
csv_header = {'time', ...
    'th1', 'th1_dot', 'th1_ddot', 'T1', ... % Giunto 1
    'th2', 'th2_dot', 'th2_ddot', 'T2', ... % Giunto 2
    'x_ee', 'y_ee', 'vx_ee', 'vy_ee', 'ax_ee', 'ay_ee', ... % End-effector
    'xtarget', 'ytarget', ...
    'Kp', 'Kd', 'linear_angle'};
fid = fopen(csv_filename, 'w');
fprintf(fid, '%s,', csv_header{1:end-1});
fprintf(fid, '%s\n', csv_header{end});
fclose(fid);

% Passa il nome del file al Plotter tramite la struct p
p.csv_filename = csv_filename;

Plotter(p);
%% ghiaia modulo di Young 2.9*10^10   poisson ratio 0.2  acciaio ghiaia0.61

