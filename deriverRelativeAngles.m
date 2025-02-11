%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Derive double pendulum dynamics %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Derive double pendulum equations of motion, write to Thdotdot1 and
%Thdotdot2 matlab function files. THIS VERSION DERIVES IT WITH THE ELBOW
%ANGLE RELATIVE TO THE UPPER ARM ANGLE.

%Parameters symbolically.
syms m1 m2 I1 I2 g l1 l2 d1 d2 th1 th2 thdot1 thdot2 thdotdot1 thdotdot2 er1 er2 eth1 eth2 T1 T2 real

%Unit vectors, cartesian
i = [1 0 0]';
j = [0 1 0]';
k = [0 0 1]';

%Rotating reference frames
er1 = [-sin(th1), cos(th1), 0]'; %Just changed it so 0 is upright.
er2 = [-sin(th2+th1), cos(th2+th1), 0]';

eth1 = [-cos(th1),-sin(th1),0]';
eth2 = [-cos(th2+th1),-sin(th2+th1),0]';

%Vectors to significant points
ra_c1 = d1*er1; %A is fixed point, B is the elbow, c1 and c2 are COMs, e end effector
rb_c2 = d2*er2;
rb_e = l2*er2;
ra_b = l1*er1;
ra_c2 = ra_b + rb_c2;
ra_e = ra_b + rb_e;

matlabFunction(ra_e, 'file', 'ForwardKin');

%Velocities
Vc1 = d1*thdot1*eth1;
VB = l1*thdot1*eth1;
Vc2 = VB + d2*(thdot2+thdot1)*eth2;
Ve = VB + l2*(thdot2+thdot1)*eth2;

%Accelerations
Ac1 = d1*thdotdot1*eth1 - d1*thdot1^2*er1;
AB = l1*thdotdot1*eth1 - l1*thdot1^2*er1;
Ac2 = d2*(thdotdot2+thdotdot1)*eth2 - d2*(thdot1 + thdot2)^2*er2  + AB;

%Force at end effector
syms Fdx Fdy real
Fd = [Fdx, Fdy, 0]';

%AMB for just link 2:
M_B = cross(rb_c2,-m2*g*j)+T2*k     + cross(rb_e,Fd); %Last term is force at end effector.
Hdot2 = I2*(thdotdot2+thdotdot1)*k + cross(rb_c2, m2*Ac2);

eqn2forthdotdot2 = solve(dot(Hdot2 - M_B,k),thdotdot2);

%AMB for whole thing:
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% Active Impedence Control %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

J = jacobian(Ve,[thdot1,thdot2]);

syms xt yt xdott ydott real

Kp_min = 1;  % Minimo per Kp
Kp_max = 100; % Massimo per Kp
Kd_min = 0.1;  % Minimo per Kd
Kd_max = 50;   % Massimo per Kd

gamma_p = 10;
gamma_d = 10;

Kp = @(p_err) Kp_min + (Kp_max - Kp_min) * (1 - exp(-gamma_p * p_err));
Kd = @(v_err) Kd_min + (Kd_max - Kd_min) * (1 - exp(-gamma_d * v_err));

zt = [xt yt 0 ]'; 
ztdot = [xdott ydott 0]';

position_error = norm(zt - ra_e);
velocity_error = norm(ztdot - J*[thdot1 thdot2]');

Kp_value = Kp(position_error);
Kd_value = Kd(velocity_error);

Ta = J' * (Kp_value * (zt - ra_e) + Kd_value * (ztdot - J * [thdot1 thdot2]'));

matlabFunction(Ta, 'file', 'ActiveImpedanceControl');

