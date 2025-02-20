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
%Create the figure object and set the callback functions.
f = figure;
set(f,'WindowButtonMotionFcn','');

figData.xtarget = [];
figData.ytarget = [];
figData.Fx = [];
figData.Fy = [];
figData.xend = [];
figData.yend = [];
figData.fig = f;
figData.tarControl = true;

%%%%%%%% 1st Subplot -- the pendulum animation %%%%%%%%
figData.simArea = subplot(1,1,1); %Eliminated other subplots, but left this for syntax consistency.
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
tmeter1 = text(0.6,-3.2,'0.00','FontSize',22,'Color', 'r');  % Torque T1
tmeter2 = text(3,-3.2,'0.00','FontSize',22,'Color', 'b');  % Torque T2

% Aggiungi i testi per visualizzare Kp_value e Kd_value
kpText = text(0.6, -4.4, 'Kp: 0.00', 'FontSize', 18, 'Color', 'm');
kdText = text(3, -4.4, 'Kd: 0.00', 'FontSize', 18, 'Color', 'c');

%Position error and velocity error
posErrorText = text(0.6,-3.6, 'Pos Error: 0.00 m', 'FontSize', 18, 'Color', 'g');
velErrorText = text(0.6,-4.0, 'Vel Error: 0.00 m/s', 'FontSize', 18, 'Color', 'k');

%Target Pt.
targetPt = plot(p.xtarget,p.ytarget,'xr','MarkerSize',18);
hold off

%Make the whole window big for handy viewing:
screenSize = get(0, 'ScreenSize');
figWidth = 800;
figHeight = 600;
figX = (screenSize(3) - figWidth) / 2;
figY = (screenSize(4) - figHeight) / 2;

set(f, 'units', 'pixels', 'position', [figX figY figWidth figHeight])
set(f,'Color',[1,1,1]);

% Turn the axis off
ax = get(f,'Children');
set(ax,'Visible','off');

% Create button
btn = uicontrol('Style', 'pushbutton', 'String', 'Start Trajectory', 'Position', [350, 50, 100, 30], 'Callback', @startStopCallback);

%Animation plot loop -- Includes symplectic integration now.
z1 = p.init;
told = 0;

set(f,'UserData',figData);

tic %Start the clock
while (ishandle(f))
    figData = get(f,'UserData');
    %%%% INTEGRATION %%%%
    tnew = toc;
    dt = tnew - told;
    
    %Old velocity and position
    xold = [z1(1),z1(3)];
    vold = [z1(2),z1(4)];
   
    %Call RHS given old state
    [zdot1, T1, T2] = FullDyn(tnew,z1,p);
    vinter1 = [zdot1(1),zdot1(3)];
    ainter = [zdot1(2),zdot1(4)];
    
    vinter2 = vold + ainter*dt; %Update velocity based on old RHS call
    
    %Update position.
    xnew = xold + vinter2*dt;
    vnew = (xnew-xold)/dt;
    
    z2 = [xnew(1) vnew(1) xnew(2) vnew(2)];

    % Limit the angle of the second link
    max_angle = p.init(3) + p.init(3) / 2; % Maximum angle for the second link
    if z2(3) > max_angle || z2(3) < -max_angle
        z2(3) = max_angle;
    end

    z1 = z2;
    told = tnew;
    %%%%%%%%%%%%%%%%%%%%

    % Update the target based on the trajectory
    if p.isActive
        [p.xtarget, p.ytarget] = p.trajectory(tnew);
        set(targetPt,'xData',p.xtarget); %Change the target point graphically.
        set(targetPt,'yData',p.ytarget);
    end

    %If there are new mouse click locations, then set those as the new
    %target.
    if ~isempty(figData.xtarget)
    p.xtarget = figData.xtarget;
    end
    
    if ~isempty(figData.ytarget)
    p.ytarget = figData.ytarget;
    end

    %When you hit a key, it changes to force mode, where the mouse will
    %pull things.
    ra_e = ForwardKin(p.l1,p.l2,z1(1),z1(3));
    figData.xend = ra_e(1);
    figData.yend = ra_e(2);
    set(f,'UserData',figData);
    
    if ~isempty(figData.Fx)
    p.Fx = figData.Fx;
    end
    if ~isempty(figData.Fy)
    p.Fy = figData.Fy;
    end
    
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

    % Calculate position and velocity errors
    position_error = norm([p.xtarget - figData.xend, p.ytarget - figData.yend]);  % Position error
    velocity_error = norm([0 - (z1(2)), 0 - (z1(4))]);  % Assuming desired velocity is 0
    
    % Calculate Kp and Kd values
    Kp_value = p.Kp(position_error);
    Kd_value = p.Kd(velocity_error);

    % Update position error and velocity error display
    set(posErrorText, 'string', strcat('Pos Error: ', num2str(position_error, 2), ' m'));  % Position error display
    set(velErrorText, 'string', strcat('Vel Error: ', num2str(velocity_error, 2), ' m/s'));  % Velocity error display
    
    % Update Kp e Kd display
    set(kpText, 'string', strcat('Kp: ', num2str(Kp_value, 2)));
    set(kdText, 'string', strcat('Kd: ', num2str(Kd_value, 2)));
    drawnow;
end
function startStopCallback(~, ~)
    p.isActive = ~p.isActive; % Alterna tra true e false
    if p.isActive
        set(btn, 'String', 'Stop Trajectory');
    else
        set(btn, 'String', 'Start Trajectory');
    end
end
end