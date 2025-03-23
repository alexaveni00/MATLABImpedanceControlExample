Kp_min = 150;  % Minimo per Kp
Kp_max = 375; % Massimo per Kp
Kd_min = 25;  % Minimo per Kd
Kd_max = 100;   % Massimo per Kd

terrainParams.soft.k = 500;     % N/m^(3/2)
terrainParams.soft.c = 150;     % N·s/m^(5/2)
terrainParams.soft.offset = 0.05;

terrainParams.hard.k = 1500;    % N/m^(3/2)
terrainParams.hard.c = 200;     % N·s/m^(5/2)
terrainParams.hard.offset = 0.001;

terrainParams.soft.n = 1.5;
terrainParams.hard.n = 1.5;

terrainLine1 = [-0.125; 0; 0.2];
init = [-3/4*pi    0.0    -pi/2 0.0];
l1 = 1; 
l2 = 1;
g = 9.81;
m1 = 1.5;
m2 = 1.5;
endZ = ForwardKin(l1,l2,init(1),init(3));
terrainParams.y_surface = endZ(2);    % Altezza di riferimento del terreno
