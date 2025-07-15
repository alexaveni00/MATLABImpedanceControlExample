%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Animate the acrobot after the MAIN script has been run.
%
%   Matthew Sheen, 2014
%
%   Note: data is passed from this to the callback functions in the figure
%   object's UserData field.
%   For compatibility with 2014a and earlier, I use set/get instead of the
%   object.field notation.
%   Can also be done with global vars as in the backup version. I hate
%   global variables so, this version is the result.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Plotter(p)
% Valore iniziale dell'angolo della traiettoria lineare (in radianti)
linear_angle = 0;
close all

% === DEBUG: mostra la X rossa sul target solo se attivo ===
DEBUG_SHOW_TARGET_X = false; % Imposta a true per vedere la X rossa
DEBUG_TRAJECTORY_TRACE = true; % imposta a true per attivare la traccia

%Playback speed:
% playback = p.animationSpeed;

%Name the whole window and define the mouse callback function
f = figure;
figData.xtarget = [];
figData.ytarget = [];
figData.xend = [];
figData.yend = [];
figData.fig = f;
figData.tarControl = true;
set(f, 'ToolBar', 'none');
set(f, 'WindowButtonMotionFcn', []);
set(f, 'WindowButtonDownFcn',   []);
set(f, 'WindowButtonUpFcn',     []);

%%%%%%%% 1st Subplot -- the pendulum animation %%%%%%%
figData.simArea = subplot(1,1,1); %Eliminated other subplots, but left this for syntax consistency.
figData.simArea.Toolbar = []; % Disable the toolbar for this subplot
axis equal
hold on

%Create pendulum link1 object:
width1 = p.l1*0.05;
xdat1 = 0.5*width1*[-1 1 1 -1];
ydat1 = p.l1*[0 0 1 1];
link1 = patch(xdat1,ydat1, [0 0 0 0],'r');

%Create pendulum link2 object:
width2 = p.l2*0.05;
xdat2 = 0.5*width2*[-1 1 1 -1];
ydat2 = p.l2*[0 0 1 1];
link2 = patch(xdat2,ydat2, [0 0 0 0],'b');
axis([-3.5 3.5 -3.6 3.6]);

%Dots for the hinges:
h1 = plot(0,0,'.k','MarkerSize',40); %First link anchor
h2 = plot(0,0,'.k','MarkerSize',40); %link1 -> link2 hinge

%Timer label:
timer = text(-3.2,-3.2,'0.00','FontSize',28);

%Torque meters on screen
tmeter1 = text(0.6,-3.2,'0.00','FontSize',22,'Color', 'r');
tmeter2 = text(2.2,-3.2,'0.00','FontSize',22,'Color', 'b');


%Target Pt.
if DEBUG_SHOW_TARGET_X
    targetPt = plot(p.xtarget,p.ytarget,'xr','MarkerSize',30);
else
    targetPt = plot(nan, nan, 'xr', 'MarkerSize', 30, 'Visible', 'off');
end

% Traccia del target (debug, creata una sola volta)
traceX = [];
traceY = [];
tracePlot = plot(nan, nan, 'r.', 'MarkerSize', 3, 'DisplayName', 'Target Trace');

hold off

%Make the whole window big for handy viewing:
set(f, 'units', 'inches', 'position', [5 5 10 9])
set(f,'Color',[1,1,1]);

% Centrare la finestra e impostare dimensioni 800x800
screenSize = get(0, 'ScreenSize');
figWidth = 800;
figHeight = 600;
figX = (screenSize(3) - figWidth) / 2;
figY = (screenSize(4) - figHeight) / 2;
set(f, 'units', 'pixels', 'position', [figX, figY, figWidth, figHeight]);
set(f,'Color',[1,1,1]);

% Turn the axis off
ax = get(f,'Children');
set(ax,'Visible','off');

%Animation plot loop -- Includes symplectic integration now.
z1 = p.init;
told = 0;

set(f,'UserData',figData);

tic %Start the clock
% === PARAMETRI TRAIETTORIA SEMICIRCONFERENZA ===
autoTrajectory = true; % Imposta a true per attivare la traiettoria automatica
raggio = 0.7; % raggio della semicirconferenza
vel_angolare = pi/2; % velocità angolare [rad/s]

% Centro della semicirconferenza
x_c = p.xtarget;
y_c = p.ytarget;

% Tracker per Kp, Kd, velocità end-effector
kpText = text(-3.2, -3.6, 'Kp: 0.00', 'FontSize', 18, 'Color', 'm');
kdText = text(-3.2, -4.0, 'Kd: 0.00', 'FontSize', 18, 'Color', 'c');
velText = text(0.6, -4.0, 'Vel: [0.00, 0.00]', 'FontSize', 18, 'Color', 'g');

% === Variabile di stato per la traiettoria (usata con setappdata/getappdata) ===
setappdata(f, 'traj_active', false);
setappdata(f, 'traj_theta', 0);

% Pulsante per avviare/ripartire la traiettoria
btn = uicontrol('Style', 'pushbutton', 'String', 'Avvia Traiettoria', ...
    'Position', [20 20 150 40], 'FontSize', 12, 'Callback', {@startTrajectory, f});
set(btn, 'Enable', 'on');

% Slider per regolare l'angolo della traiettoria lineare

slider_width = 250;
slider_height = 20;
slider_x = (figWidth - slider_width) / 2;
slider_y = figHeight - 40; % 40 pixel dal bordo superiore
slider = uicontrol('Style', 'slider', 'Min', -pi/2, 'Max', pi/2, 'Value', linear_angle, ...
    'Position', [slider_x slider_y slider_width slider_height], 'Callback', @(src,~) setappdata(f, 'linear_angle', get(src,'Value')));
addlistener(slider, 'Value', 'PostSet', @(src,evt) setappdata(f, 'linear_angle', get(slider,'Value')));
setappdata(f, 'linear_angle', linear_angle);


% Imposta valore iniziale del ground_angle in appdata
set(f,'UserData',figData);

set(link1, 'HitTest','off', 'PickableParts','none');
set(link2, 'HitTest','off', 'PickableParts','none');
set(h1,    'HitTest','off', 'PickableParts','none');
set(h2,    'HitTest','off', 'PickableParts','none');
set(targetPt, 'HitTest','off', 'PickableParts','none');
set(tracePlot, 'HitTest', 'off', 'PickableParts', 'none');

while (ishandle(f))
    figData = get(f,'UserData');
    %%%% INTEGRATION %%%%
    tnew = toc;
    dt = tnew - told;

    % LOG DATI SU CSV: ora tutto dopo FullDyn
    
    %If there are new mouse click locations, then set those as the new
    %target.
    if ~isempty(figData.xtarget)
        p.xtarget = figData.xtarget;
    end
    if ~isempty(figData.ytarget)
        p.ytarget = figData.ytarget;
    end
    % Aggiorna la X rossa solo se il debug è attivo
    if DEBUG_SHOW_TARGET_X
        set(targetPt,'xData',p.xtarget); %Change the target point graphically.
        set(targetPt,'yData',p.ytarget);
        set(targetPt, 'Visible', 'on');
    else
        set(targetPt, 'Visible', 'off');
    end

    % ======= INTEGRAZIONE DINAMICA =======
    %Old velocity and position
    xold = [z1(1),z1(3)];
    vold = [z1(2),z1(4)];
    p.fig = f; % Passa il handle della figura

    % === TRAIETTORIA AUTOMATICA ===
    traj_active = getappdata(f, 'traj_active');
    traj_theta = getappdata(f, 'traj_theta');
    if autoTrajectory && traj_active
        T_semi = pi / vel_angolare;
        linear_angle = getappdata(f, 'linear_angle');
        % Ruota il centro della semicirconferenza per coerenza con la traiettoria lineare
        x_c_rot = x_c;
        y_c_rot = y_c;
        if traj_theta <= pi
            [p.Kp, p.Kd] = computeKpKd(p.init(4));
            % Durante la semicirconferenza: Kp e Kd restano fissi (valore iniziale)
            [x0, y0] = SemicircleTrajectory(traj_theta/vel_angolare, 0, 0, raggio, vel_angolare);
            x_traj = x_c_rot + cos(linear_angle)*x0 - sin(linear_angle)*y0;
            y_traj = y_c_rot + sin(linear_angle)*x0 + cos(linear_angle)*y0;
        else
            % Durante la traiettoria rettilinea: aggiorna Kp e Kd dinamicamente
            [p.Kp, p.Kd] = computeKpKd(z1(4));
            t_diam = (traj_theta - pi) / vel_angolare;
            T_diam = T_semi;
            t_norm = min(t_diam / T_diam, 1);
            [x_traj, y_traj] = LinearTrajectory(t_norm, x_c_rot, y_c_rot, raggio, linear_angle);
        end
        traj_theta = traj_theta + vel_angolare * dt;
        if traj_theta >= pi + T_semi * vel_angolare
            traj_theta = 0;
        end
        figData.xtarget = x_traj;
        figData.ytarget = y_traj;

        % Debug: mostra la x
        if DEBUG_SHOW_TARGET_X
            set(targetPt,'xData',figData.xtarget);
            set(targetPt,'yData',figData.ytarget);
        end
        setappdata(f, 'traj_theta', traj_theta);
        % Debug: lascia traccia del target
        if DEBUG_TRAJECTORY_TRACE
            traceX(end+1) = x_traj;
            traceY(end+1) = y_traj;
            plotTargetTrace(tracePlot, traceX, traceY);
        end
    end
    [zdot1, T1, T2] = FullDyn(z1,p);
    % === LOG DATI SU CSV ===
    if isfield(p, 'csv_filename') && ~isempty(p.csv_filename)
        th1 = z1(1); th1_dot = z1(2);
        th2 = z1(3); th2_dot = z1(4);
        th1_ddot = zdot1(2);
        th2_ddot = zdot1(4);
        ee = ForwardKin(p.l1, p.l2, th1, th2);
        x_ee = ee(1); y_ee = ee(2);
        J = JacobianEndeffector(p.l1, p.l2, th1, th2);
        qdot = [th1_dot; th2_dot];
        v_ee = J * qdot;
        % Accelerazione end-effector
        delta = 1e-6;
        J_next = JacobianEndeffector(p.l1, p.l2, th1 + th1_dot*delta, th2 + th2_dot*delta);
        Jdot = (J_next - J) / delta;
        qddot = [th1_ddot; th2_ddot];
        a_ee = J * qddot + Jdot * qdot;
        Kp = p.Kp; Kd = p.Kd;
        xtarget = p.xtarget;
        ytarget = p.ytarget;
        linear_angle = getappdata(f, 'linear_angle');
        log_time = tnew;
        fid = fopen(p.csv_filename, 'a');
        if fid ~= -1
            % tempo, angolo1, vel1, acc1, torque1, angolo2, vel2, acc2, torque2, x_ee, y_ee, vx_ee, vy_ee, ax_ee, ay_ee, xtarget, ytarget, kp, kd, linear_angle
            fprintf(fid, '%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
                log_time, th1, th1_dot, th1_ddot, T1, th2, th2_dot, th2_ddot, T2, x_ee, y_ee, v_ee(1), v_ee(2), a_ee(1), a_ee(2), xtarget, ytarget, Kp, Kd, linear_angle);
            fclose(fid);
        end
    end
    vinter1 = [zdot1(1),zdot1(3)];
    ainter = [zdot1(2),zdot1(4)];
    vinter2 = vold + ainter*dt;
    xnew = xold + vinter2*dt;
    vnew = (xnew-xold)/dt;
    % Calcola la velocità end-effector attuale (modulo)
    z2 = [xnew(1) vnew(1) xnew(2) vnew(2)];
    z1 = z2;
    told = tnew;
    
    %When you hit a key, it changes to force mode, where the mouse will
    %pull things.
    ra_e = ForwardKin(p.l1,p.l2,z1(1),z1(3));
    figData.xend = ra_e(1);
    figData.yend = ra_e(2);
    set(f,'UserData',figData);
    
    tstar = told; %Get the time (used during this entire iteration)
    
    %On screen timer.
    set(timer,'string',strcat(num2str(tstar,3),'s'))
    zstar = z1;%interp1(time,zarray,tstar); %Interpolate data at this instant in time.
    
    %Rotation matrices to manipulate the vertices of the patch objects
    %using theta1 and theta2 from the output state vector.
    rot1 = [cos(zstar(1)), -sin(zstar(1)); sin(zstar(1)),cos(zstar(1))]*[xdat1;ydat1];
    set(link1,'xData',rot1(1,:))
    set(link1,'yData',rot1(2,:))
    
    rot2 = [cos(zstar(3)+zstar(1)), -sin(zstar(3)+zstar(1)); sin(zstar(3)+zstar(1)),cos(zstar(3)+zstar(1))]*[xdat2;ydat2];
    
    set(link2,'xData',rot2(1,:)+(rot1(1,3)+rot1(1,4))/2) %We want to add the midpoint of the far edge of the first link to all points in link 2.
    set(link2,'yData',rot2(2,:)+(rot1(2,3)+rot1(2,4))/2)
    
    %Change the hinge dot location
    set(h2,'xData',(rot1(1,3)+rot1(1,4))/2)
    set(h2,'yData',(rot1(2,3)+rot1(2,4))/2)
    
    %Show torques on screen (text only atm) update for time series later.
    set(tmeter1,'string',strcat(num2str(T1,2),' Nm'));
    set(tmeter2,'string',strcat(num2str(T2,2),' Nm'));
        
    % Aggiorna i tracker a video
    set(kpText, 'String', sprintf('Kp: %.2f', p.Kp));
    set(kdText, 'String', sprintf('Kd: %.2f', p.Kd));
    % Calcola velocità end-effector
    J = JacobianEndeffector(p.l1, p.l2, z1(1), z1(3));
    qdot = [z1(2); z1(4)];
    v_ee = J * qdot;
    set(velText, 'String', sprintf('Vel: [%.2f, %.2f]', v_ee(1), v_ee(2)));
    drawnow;
end
end

function startTrajectory(~,~,f)
    setappdata(f, 'traj_active', true);        % attiva traiettoria
    setappdata(f, 'traj_theta', 0);
    set(findobj('String','Avvia Traiettoria'), 'Enable', 'off');
end


function plotTargetTrace(tracePlot, traceX, traceY)
% plotTargetTrace - Aggiorna la traccia del target nella figura
%   tracePlot: handle al plot della traccia
%   traceX, traceY: vettori delle posizioni target
    if nargin < 3
        error('plotTargetTrace richiede tracePlot, traceX, traceY');
    end
    if ishandle(tracePlot)
        set(tracePlot, 'XData', traceX, 'YData', traceY);
    end
end