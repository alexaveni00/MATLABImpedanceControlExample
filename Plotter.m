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
close all

% === DEBUG: mostra la X rossa sul target solo se attivo ===
DEBUG_SHOW_TARGET_X = false; % Imposta a true per vedere la X rossa

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
DEBUG_TRAJECTORY_TRACE = true; % imposta a true per attivare la traccia
traceX = [];
traceY = [];
tracePlot = plot(nan, nan, 'r.', 'MarkerSize', 10, 'DisplayName', 'Target Trace');

% Vincolo: retta orizzontale passante per il punto di inizio traiettoria
constraintLine = plot(nan, nan, 'k-', 'LineWidth', 1, 'DisplayName', 'Constraint Line');
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
y_c = p.ytarget; % sposta in basso

% Tracker per Kp, Kd, velocità end-effector
kpText = text(-3.2, -3.6, 'Kp: 0.00', 'FontSize', 18, 'Color', 'm');
kdText = text(-3.2, -4.0, 'Kd: 0.00', 'FontSize', 18, 'Color', 'c');
velText = text(0.6, -4.0, 'Vel: [0.00, 0.00]', 'FontSize', 18, 'Color', 'g');

% === Variabile di stato per la traiettoria (usata con setappdata/getappdata) ===
setappdata(f, 'traj_active', false);
setappdata(f, 'traj_theta', 0);
setappdata(f, 'contatto_attivo', false);  % inizialmente il contatto è disabilitato


% Pulsante per avviare/ripartire la traiettoria
btn = uicontrol('Style', 'pushbutton', 'String', 'Avvia Traiettoria', ...
    'Position', [20 20 150 40], 'FontSize', 12, 'Callback', {@startTrajectory, f});
set(btn, 'Enable', 'on');

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

    % === TRAIETTORIA AUTOMATICA ===
    traj_active = getappdata(f, 'traj_active');
    traj_theta = getappdata(f, 'traj_theta');
    if autoTrajectory && traj_active
        [x_traj, y_traj] = SemicircleTrajectory(traj_theta/vel_angolare, x_c, y_c, raggio, vel_angolare);
        traj_theta = traj_theta + vel_angolare * dt;
       
            figData.xtarget = x_traj;
            figData.ytarget = y_traj;
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
    [zdot1, T1, T2] = FullDyn(z1,p);
    vinter1 = [zdot1(1),zdot1(3)];
    ainter = [zdot1(2),zdot1(4)];
    vinter2 = vold + ainter*dt;
    xnew = xold + vinter2*dt;
    vnew = (xnew-xold)/dt;
    % Calcola la velocità end-effector attuale (modulo)
    [p.Kp, p.Kd] = computeKpKd(z1(4)); % Aggiorna Kp e Kd in base alla velocità angolare del giunto del "ginocchio" (thdot2)
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
    figData = get(f, 'UserData');
    setappdata(f, 'enable_constraint', true);  % abilita vincolo orizzontale
    setappdata(f, 'traj_active', true);        % attiva traiettoria
    setappdata(f, 'traj_theta', 0);
    setappdata(f, 'contatto_attivo', false);   % disattiva contatto fino a fine traiettoria
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