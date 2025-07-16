% plot_kd_vs_th2dot.m
% Mostra la variazione di Kd e della velocità angolare del secondo giunto nel tempo

filename = 'log_dati_simulazione_kp500_kd20-70.csv';
data = readtable(filename);

figure('Name','Kd e Velocità Angolare Giunto 2 vs Tempo','Position',[200 200 900 500]);

yyaxis left;
plot(data.time, data.Kd, 'b-', 'DisplayName','Kd');
ylabel('Kd');

yyaxis right;
plot(data.time, data.th2_dot, 'r-', 'DisplayName','th2\_dot');
ylabel('Velocità angolare giunto 2 [rad/s]');

xlabel('Tempo [s]');
title('Kd e Velocità Angolare Giunto 2 vs Tempo');
grid on;
legend({'Kd','th2\_dot'});
