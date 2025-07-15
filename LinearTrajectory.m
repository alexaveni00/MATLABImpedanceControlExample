function [x, y] = LinearTrajectory(t, x_c, y_c, raggio, linear_angle)
% LinearTrajectory - Movimento lineare lungo il diametro da sinistra a destra
%   t: parametro normalizzato [0,1] (0 inizio, 1 fine)
%   x_c, y_c: centro della semicirconferenza
%   raggio: raggio della semicirconferenza
%   Restituisce x, y posizione target

% Clamp t tra 0 e 1
t = max(0, min(1, t));

x_start = x_c - raggio * cos(linear_angle);
x_end = x_c + raggio * cos(linear_angle);
y_start = y_c - raggio * sin(linear_angle);
y_end = y_c + raggio * sin(linear_angle);

x = (1 - t) * x_start + t * x_end;
y = (1 - t) * y_start + t * y_end;