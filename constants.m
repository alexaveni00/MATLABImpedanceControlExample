Kp_min = 150;  % Minimo per Kp
Kp_max = 375; % Massimo per Kp
Kd_min = 25;  % Minimo per Kd
Kd_max = 100;   % Massimo per Kd

terrainParams.soft.k = 500;     % N/m^(3/2)
terrainParams.soft.c = 150;     % N·s/m^(5/2)

terrainParams.hard.k = 1500;    % N/m^(3/2)
terrainParams.hard.c = 400;     % N·s/m^(5/2)

terrainParams.step.k = 2500;    % N/m^(3/2)
terrainParams.step.c = 200;     % N·s/m^(5/2)

terrainParams.soft.n = 1.5;
terrainParams.hard.n = 1.5;
terrainParams.step.n = 1.5;

init = [-3/4*pi    0.0    -pi/2 0.0];
l1=1; 
l2=1;
endZ = ForwardKin(l1,l2,init(1),init(3));
terrainParams.y_surface = endZ(2);    % Altezza di riferimento del terreno
