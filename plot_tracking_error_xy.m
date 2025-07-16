% plot_tracking_error_xy.m
% Mostra la differenza tra traiettoria desiderata e attuale in x e y (non nel tempo)


filename = 'logdatisimulazionekp25kd20-30.csv';
data = readtable(filename);

% Calcola errore di tracking
err_x = data.xtarget - data.x_ee;
err_y = data.ytarget - data.y_ee;

figure('Name',['Traiettoria desiderata vs attuale - ', filename],'Position',[200 200 700 600]);
plot(data.xtarget, data.ytarget, 'r--', 'LineWidth',1.5, 'DisplayName','Desiderata'); hold on;
plot(data.x_ee, data.y_ee, 'b-', 'LineWidth',1.5, 'DisplayName','Attuale');
legend('show','Location','best');
xlabel('x [m]'); ylabel('y [m]');
title(['Traiettoria desiderata vs attuale'],'Interpreter','tex');
axis equal; grid on;

% Allarga la view aggiungendo un margine automatico
all_x = [data.xtarget; data.x_ee];
all_y = [data.ytarget; data.y_ee];
margin_x = 0.08 * (max(all_x) - min(all_x));
margin_y = 0.08 * (max(all_y) - min(all_y));
xlim([min(all_x)-margin_x, max(all_x)+margin_x]);
ylim([min(all_y)-margin_y, max(all_y)+margin_y]);
