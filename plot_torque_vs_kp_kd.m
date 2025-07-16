% plot_torque_vs_kp_kd.m
% Mostra come variano le torque T1 e T2 in funzione di Kp e Kd


filename = 'logdatisimulazionekp25kd20-30.csv';
data = readtable(filename);

% Calcolo la media dei valori (ignorando i NaN)
mean_T1 = mean(data.T1,'omitnan');
mean_T2 = mean(data.T2,'omitnan');


figure('Name',['Torque vs Kp/Kd - ', filename],'Position',[200 200 1000 500]);

yyaxis left;
yyaxis right;
xlabel('Tempo [s]');
title('Giunto 1: Torque, Kp, Kd vs Tempo'); grid on;



subplot(1,2,1);
yyaxis left;
plot(data.time, data.T1, 'b-', 'DisplayName','T1'); hold on;
plot([data.time(1), data.time(end)], [mean_T1, mean_T1], 'k--', 'LineWidth',1.2, 'DisplayName','Media T1');
ylabel('T_1 [Nm]');
yyaxis right;
plot(data.time, data.Kd, 'g-', 'DisplayName','Kd');
ylabel('Kd');
ylim([15 35]);
xlabel('Tempo [s]');
title('Giunto 1: Torque, Kd vs Tempo'); grid on;
legend({'T1','Media T1','Kd'});

yyaxis left;
yyaxis right;
xlabel('Tempo [s]');
title('Giunto 2: Torque, Kp, Kd vs Tempo'); grid on;



subplot(1,2,2);
yyaxis left;
plot(data.time, data.T2, 'b-', 'DisplayName','T2'); hold on;
plot([data.time(1), data.time(end)], [mean_T2, mean_T2], 'k--', 'LineWidth',1.2, 'DisplayName','Media T2');
ylabel('T_2 [Nm]');
yyaxis right;
plot(data.time, data.Kd, 'g-', 'DisplayName','Kd');
ylabel('Kd');
ylim([15 35]);
xlabel('Tempo [s]');
title('Giunto 2: Torque, Kd vs Tempo'); grid on;
legend({'T2','Media T2','Kd'});
