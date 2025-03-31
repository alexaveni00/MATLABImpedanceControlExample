Kp_min = 100;  % Minimo per Kp
Kp_max = 500; % Massimo per Kp
Kd_min = 25;  % Minimo per Kd
Kd_max = 100;   % Massimo per Kd

terrainParams.soft.k = 500;
terrainParams.soft.c = 200;
terrainParams.soft.n = 1.5;
terrainParams.soft.offset = 0.05; % Offset per attivare contatto leggero con terreno soft

terrainParams.hard.k = 3000;
terrainParams.hard.c = 200;
terrainParams.hard.n = 1.5;
terrainParams.hard.offset = 0.001; % Offset per attivare contatto leggero con terreno hard

terrainLine1.soft = -0.1;
terrainLine1.hard = 0;

init = [-3/4*pi    0.0    -pi/2 0.0];
l1 = 1; 
l2 = 1;
g = 9.81;
m1 = 1.5;
m2 = 1.5;
endZ = ForwardKin(l1,l2,init(1),init(3));
terrainParams.y_surface = endZ(2);    % Altezza di riferimento del terreno
