function [x, y] = LinearTrajectory(t, x_c, y_c, raggio)
% LinearTrajectory - Movimento lineare lungo il diametro da sinistra a destra
%   t: parametro normalizzato [0,1] (0 inizio, 1 fine)
%   x_c, y_c: centro della semicirconferenza
%   raggio: raggio della semicirconferenza
%   Restituisce x, y posizione target

% Clamp t tra 0 e 1
t = max(0, min(1, t));

x_start = x_c - raggio;
x_end = x_c + raggio;
y_diam = y_c;

x = (1 - t) * x_start + t * x_end;
y = y_diam;