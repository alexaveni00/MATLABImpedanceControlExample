% plot_kp_kd_traj.m
% Mostra Kp e Kd nel tempo e la quota dell'end-effector nel tempo


filename = 'log_dati_simulazione_kp500_kd20-70.csv';
data = readtable(filename);

figure('Name','Kp/Kd e Traiettoria','Position',[200 200 900 700]);

subplot(2,1,1);
plot(data.time, data.Kp, 'b-', 'DisplayName','Kp'); hold on;
plot(data.time, data.Kd, 'r-', 'DisplayName','Kd');
xlabel('Tempo [s]'); ylabel('Valore');
title('Kp e Kd nel tempo'); legend; grid on;

subplot(2,1,2);
plot(data.time, data.y_ee, 'k-');
xlabel('Tempo [s]'); ylabel('y_{ee} [m]');
title('Variazione di y_{ee} nel tempo'); grid on;
