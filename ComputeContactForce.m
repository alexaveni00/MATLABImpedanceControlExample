function F_contact = ComputeContactForce(y_end_effector, vy_end_effector, terrainType, terrainParams)
    % ComputeContactForce - Calcola la forza di contatto usando il modello Hunt-Crossley
    %
    % INPUT:
    % y_end_effector: posizione verticale dell’end-effector [m]
    % vy_end_effector: velocità verticale dell’end-effector [m/s]
    % terrainType: tipo di terreno ('soft', 'hard')
    % terrainParams: struttura con parametri k, c, n, y_surface, offset
    %
    % OUTPUT:
    % F_contact: forza verticale risultante [N]
    
    persistent prev_penetration prev_F_damping

    if isempty(prev_penetration)
        prev_penetration = 0;
    end
    if isempty(prev_F_damping)
        prev_F_damping = 0;
    end

    % Estrai parametri
    k = terrainParams.(terrainType).k;
    c = terrainParams.(terrainType).c;
    n = terrainParams.(terrainType).n;
    y_surface = terrainParams.y_surface;
    offset = terrainParams.(terrainType).offset;

    % Penetrazione
    penetration = max(0, y_surface + offset - y_end_effector);

    if penetration > 0
        F_elastic = k * penetration^n;

        if penetration > prev_penetration || vy_end_effector ~= 0
            % Carico: penetrazione aumenta o il braccio scende
            F_damping = c * penetration^n * vy_end_effector;
        else
            % Scarico: rilassamento della forza di smorzamento
            relaxation_factor = 0.95;
            F_damping = relaxation_factor * prev_F_damping;
        end

        F_contact = F_elastic + F_damping;
        prev_F_damping = F_damping;
        prev_penetration = penetration;
    else
        % Nessun contatto
        F_contact = 0;
        prev_penetration = 0;
        prev_F_damping = 0;
    end
end