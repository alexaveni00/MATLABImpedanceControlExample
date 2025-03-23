function F_contact = ComputeContactForce(y_end_effector, vy_end_effector, terrainType, terrainParams, m, g)
    % ComputeContactForce - Calcola la forza di contatto usando il modello Hunt-Crossley
    %
    % INPUT:
    % y_end_effector: posizione verticale dell’end-effector [m]
    % vy_end_effector: velocità verticale dell’end-effector [m/s]
    % terrainType: tipo di terreno ('soft', 'hard')
    % terrainParams: struttura con parametri k, c, n, y_surface, offset
    % p: struttura dei parametri del robot (può contenere massa per la forza peso)
    %
    % OUTPUT:
    % F_contact: forza verticale risultante [N]
    
    persistent prev_penetration prev_force
    
    if isempty(prev_penetration)
        prev_penetration = 0;
    end
    if isempty(prev_force)
        prev_force = 0;
    end

    % Estrai parametri del terreno
    k = terrainParams.(terrainType).k;
    c = terrainParams.(terrainType).c;
    n = terrainParams.(terrainType).n;
    y_surface = terrainParams.y_surface;
    offset = terrainParams.(terrainType).offset; % Offset per attivare contatto leggero

    % Forza peso (approssimazione con somma delle masse)
    F_weight = m * g;

    % Calcolo della penetrazione con offset
    penetration = max(0, y_surface + offset - y_end_effector);
    
    % Se penetrazione > 0 → c'è contatto
    if penetration > 0
        if penetration > prev_penetration
            % Fase di caricamento (loading): modello Hunt-Crossley
            F_elastic = k * penetration^n;
            F_damping = c * penetration^n * vy_end_effector;
            F_contact = F_elastic + F_damping + F_weight;
            prev_force = F_contact;  % Salva per rilascio
        else
            % Fase di scarico (unloading): forza decrescente verso 0
            relaxation_factor = 0.95;
            F_contact = relaxation_factor * prev_force;
            prev_force = F_contact;
        end
        prev_penetration = penetration;
    else
        F_contact = 0;
        prev_penetration = 0;
        prev_force = 0;
    end
end
