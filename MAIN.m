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

% Proprietà materiale (end-effector vs suolo)
p.E1  = 210e9;    % modulo di Young della piastra (steel) [Pa] https://www.youmath.it/lezioni/fisica/dinamica/3032-modulo-di-young.html
p.nu1 = 0.3;      % coeff. di Poisson https://www.youmath.it/lezioni/fisica/dinamica/3033-coefficiente-di-poisson.html
p.R1  = 0.05;     % raggio di curvatura dell'end-effector [m]

p.hardParams = struct( ...
  'E2', 210e9, ...        % Young del suolo (acciaio) [Pa] https://www.samaterials.it/content/young's-modulus-an-overview.html
  'nu2',0.3, ...        % Poisson del suolo https://www.youmath.it/lezioni/fisica/dinamica/3033-coefficiente-di-poisson.html
  'R2', Inf, ...        % piano
  'e', 0.6 ...          % restitution
);

p.softParams = struct( ...
  'E2', 5e6, ...        % Young (ghiaia) [Pa] (https://www.samaterials.it/content/young's-modulus-an-overview.html)
  'nu2', 0.45, ...            % Poisson del suolo (ghiaia) https://www.geostru.com/help_online_2015/spw/it/index.html?database_caratteristiche_fisic.htm
  'R2', Inf, ...
  'e', 0.8 ...
);
% Avvia animazione & integrazione
Plotter(p);
%% ghiaia modulo di Young 2.9*10^10   poisson ratio 0.2  acciaio ghiaia0.61

