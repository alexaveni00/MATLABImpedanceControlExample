function J = JacobianEndeffector(l1, l2, th1, th2)
%JACOBIAN_ENDEFFECTOR Calcola il Jacobiano dell'end-effector per un braccio a 2 link
%   J = JacobianEndeffector(l1, l2, th1, th2)
%   Restituisce la matrice Jacobiana 2x2 per le coordinate (th1, th2)
    t2 = th1 + th2;
    J = [-l1*cos(th1)-l2*cos(t2), -l2*cos(t2);
          l1*sin(th1)+l2*sin(t2),  l2*sin(t2)];
end
