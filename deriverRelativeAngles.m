%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Derive double pendulum dynamics %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Derive double pendulum equations of motion, write to Thdotdot1 and
%Thdotdot2 matlab function files. THIS VERSION DERIVES IT WITH THE ELBOW
%ANGLE RELATIVE TO THE UPPER ARM ANGLE.

%Parameters symbolically.
syms m1 m2 I1 I2 g l1 l2 d1 d2 th1 th2 thdot1 thdot2 thdotdot1 thdotdot2 er1 er2 eth1 eth2 T1 T2 real

%Unit vectors, cartesian
i = [1 0 0]'; %versore asse x
j = [0 1 0]'; %versore asse y
k = [0 0 1]'; %versore asse z

% Rotazione attorno all’asse z di un angolo theta:
% Rz(theta) = [ cos(theta)  -sin(theta)   0
%             sin(theta)   cos(theta)   0
%               0          0        1 ];

%Rotating reference frames (Ruoto costantetemnte il sistema di riferimento attorno all'asse z)
er1 = [-sin(th1), cos(th1), 0]'; % Versore radiale alla direzione del primo link (Rz(theta)*j)  
eth1 = [-cos(th1),-sin(th1),0]'; % %Versore tangenziale alla direzione del primo link (Rz(theta)*(-i))

% stesso discorso per il secondo link, ma ora il sistema di riferimento è ruotato di th1
% rispetto al primo link, quindi la rotazione è di th1+th2
er2 = [-sin(th2+th1), cos(th2+th1), 0]'; % Versore radiale alla direzione del secondo link
eth2 = [-cos(th2+th1),-sin(th2+th1),0]'; %Versore tangenziale alla direzione del secondo link

%Vectors to significant points
%A is fixed point, B is the elbow, c1 and c2 are COMs, e end effector
ra_c1 = d1*er1; % Posizione del centro di massa del primo link
rb_c2 = d2*er2; % Posizione del centro di massa del secondo link rispetto al giunto 2
rb_e = l2*er2;  % Posizione dell'end-effector rispetto al giunto 2
ra_b = l1*er1;  % Posizione del giunto 2 rispetto al giunto 1
ra_c2 = ra_b + rb_c2; % Posizione del centro di massa del secondo link rispetto al giunto 1
ra_e = ra_b + rb_e;   % Posizione dell'end-effector rispetto al giunto 1

matlabFunction(ra_e, 'file', 'ForwardKin');

%Velocities
Vc1 = d1*thdot1*eth1; %Velocità del centro di massa del primo link
VB = l1*thdot1*eth1; % Velocità del giunto 2 rispetto al giunto 1
Vc2 = VB + d2*(thdot2+thdot1)*eth2; % Velocità del centro di massa del secondo link rispetto al giunto 1
Ve = VB + l2*(thdot2+thdot1)*eth2; % Velocità dell'end-effector rispetto al giunto 1

%Accelerations
Ac1 = d1*thdotdot1*eth1 - d1*thdot1^2*er1; % Accelerazione del centro di massa del primo link
AB = l1*thdotdot1*eth1 - l1*thdot1^2*er1; % Accelerazione del giunto 2 rispetto al giunto 1
Ac2 = d2*(thdotdot2+thdotdot1)*eth2 - d2*(thdot1 + thdot2)^2*er2  + AB; % Accelerazione del centro di massa del secondo link rispetto al giunto 1

%Force at end effector
syms Fdx Fdy real
Fd = [Fdx, Fdy, 0]';

%cross serve per calcolare il prodotto vettoriale tra due vettori
%AMB for just link 2:
M_B = cross(rb_c2,-m2*g*j)+T2*k     + cross(rb_e,Fd); %Last term is force at end effector.
Hdot2 = I2*(thdotdot2+thdotdot1)*k + cross(rb_c2, m2*Ac2);

eqn2forthdotdot2 = solve(dot(Hdot2 - M_B,k),thdotdot2);

%AMB for whole thing: 4.2 del libro di Kurfess:
M_A = cross(ra_c2,-m2*g*j) + cross(ra_c1,-m1*g*j) + T1*k + cross(ra_e,Fd); %Gravity for both, plus a control torque. Last term is force at end effector
Hdot1 = I2*(thdotdot2+thdotdot1)*k + cross(ra_c2, m2*Ac2) + I1*thdotdot1*k + cross(ra_c1, m1*Ac1);

eqn1forthdotdot1 = solve(dot(Hdot1 - M_A,k),thdotdot1);

%One equation for thdotdot1, one for thdotdot2.
eqn2 = simplify(solve(subs(eqn2forthdotdot2, thdotdot1, eqn1forthdotdot1)-thdotdot2,thdotdot2));
eqn1 = simplify(solve(subs(eqn1forthdotdot1, thdotdot2, eqn2forthdotdot2)-thdotdot1,thdotdot1));

%Create matlab functions for thdotdot1 and 2:
matlabFunction(eqn1, 'file', 'Thdotdot1');
matlabFunction(eqn2, 'file', 'Thdotdot2');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Gravity Compensation %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T2Eq = simplify(solve((solve(eqn1,T1)-solve(eqn2,T1)),T2));

T1Eq = simplify(subs(solve(eqn1,T1),T2,T2Eq));

matlabFunction(T1Eq, 'file', 'GravityCompT1');
matlabFunction(T2Eq, 'file', 'GravityCompT2');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Impedance control?   %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Jacobian relating end effector velocity to joint space vel
% ie Ve = J*qv

J = jacobian(Ve,[thdot1 thdot2]); %paragrafo 2.4

syms Kp Kd xt yt xdott ydott real

zt = [xt yt 0 ]'; %Trajectory tracked
ztdot = [xdott ydott 0]'; %velocity tracked
Ta = J'* (Kp*(zt - ra_e) + Kd*(ztdot - J*[thdot1 thdot2]'));

matlabFunction(Ta, 'file', 'ImpedenceControl');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Energy eqns %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Turned this off after making sure stuff worked:

% syms k1 k2 real
% 
% PE = g*dot(ra_c2,j)*m2 + g*dot(ra_c1,j)*m1;
% KE = 1/2*I1*thdot1^2 + 1/2*I2*(thdot2+thdot1)^2 + 1/2*m1*dot(Vc1,Vc1) + 1/2*m2*dot(Vc2,Vc2);
% 
% Etot = PE + KE;
% 
% matlabFunction(Etot, 'file', 'TotEnergy');