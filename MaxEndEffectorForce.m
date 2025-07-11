function Fmax = MaxEndEffectorForce(z,p)
    % 1) Jacobiano
    th1 = z(1); thdot1 = z(2);
    th2 = z(3); thdot2 = z(4);
    J = JacobianEndeffector(p.l1,p.l2,th1,th2);

    % 2) Dinamica libera: accelerazioni e torque Totale
    [zdot_free, tau1, tau2] = FullDyn(z, p);
    tau = [tau1; tau2];
    qddot = [zdot_free(2); zdot_free(4)];

    % 3) Massa, Coriolis, Gravità
    [M, C, G] = MassCoriolisGravity(...
       th1,th2,thdot1,thdot2, ...
       p.m1,p.m2,p.l1,p.l2,p.d1,p.d2,p.I1,p.I2,p.g);

    % 4) Forza esterna residua (come prima)
    w_resid = tau - (M*qddot + C + G);
    F_resid = pinv(J') * w_resid;

    % 5) Forza statica già compensata
    %    è la parte di tau che compensa G + M*qddot + C:
    F_stat = pinv(J') * (M*qddot + C + G);

    % 6) Forza totale trasferibile:
    Ftot = F_resid + F_stat;

    Fmax = Ftot(2);
end
