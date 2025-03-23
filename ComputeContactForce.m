function Fn = ComputeContactForce(y_effector, ydot_effector, terrainType, terrainParams)
    % ComputeContactForce calcola la forza di contatto basata su un modello
    % visco-elastico non lineare (Hunt-Crossley).
    %
    % INPUT:
    %   y_effector    : altezza attuale dell'end-effector
    %   ydot_effector : velocità verticale dell'end-effector (positiva verso il basso)
    %   terrainType   : 'soft', 'hard', oppure 'step'
    %   terrainParams : struttura con parametri del terreno (k, c, y_surface)
    %
    % OUTPUT:
    %   Fn            : forza di contatto (positiva verso l'alto)
    
        % Penetrazione dell'end-effector rispetto alla superficie
        if (isempty(y_effector))
            y_effector = terrainParams.y_surface;
        end

        % Parametri del terreno in base al tipo
        switch terrainType
            case 'soft'
                k = terrainParams.soft.k;
                c = terrainParams.soft.c;
                n = terrainParams.soft.n;
            case 'hard'
                k = terrainParams.hard.k;
                c = terrainParams.hard.c;
                n = terrainParams.hard.n;
            case 'step'
                k = terrainParams.step.k;
                c = terrainParams.step.c;
                n = terrainParams.step.n;
            otherwise
                error('Unknown terrain type');
        end
        y_surface = terrainParams.y_surface;
        penetration = max (0, y_surface - y_effector);
        penetration_dot = -ydot_effector; 
    
        if penetration == 0
            Fn = 0;
            return;
        end
        % Modello Hunt-Crossley
        Fn_elastic = k * penetration^n;
        Fn_damping = c * penetration^n * penetration_dot;
    
        Fn = Fn_elastic + Fn_damping;
    
        % Forza non può essere negativa
        Fn = max(Fn, 0);
        if isempty(Fn)
            Fn = 0;
        end
    end
    