function [x, y, theta] = SemicircleTrajectory(t, x_c, y_c, raggio, vel_angolare)
% Restituisce la posizione [x, y] lungo una semicirconferenza
% t: tempo
% x_c, y_c: centro
% raggio: raggio
% vel_angolare: velocità angolare
% theta: angolo attuale
    theta = vel_angolare * t;
    x = x_c + raggio * cos(theta);
    y = y_c + raggio * sin(theta);
end
