function [Kp_est, P, theta, filtered_force, filtered_displacement] = EstimateStiffness_IIR_RLS(force_input, displacement_input, p)
    % EstimateStiffness_IIR_RLS
    %
    % Questa funzione implementa un algoritmo di stima della rigidità (Kp)
    % utilizzando un filtro digitale IIR progettato nel dominio continuo,
    % trasformato al dominio discreto tramite trasformazione bilineare (Tustin)
    % e, infine, utilizzando un algoritmo di Recursive Least Squares (RLS).
    %
    % INPUT:
    % force_input          - misura della forza esterna (N)
    % displacement_input   - misura dello spostamento del manipolatore (m)
    % dt                   - intervallo di tempo di campionamento (s)
    % lambda               - fattore di dimenticanza per RLS (0 < lambda <= 1)
    % P                    - matrice di covarianza RLS (inizialmente alta)
    % theta                - vettore dei parametri stimati iniziali
    %
    % OUTPUT:
    % Kp_est               - stima della rigidità calcolata (N/m)
    % P                    - matrice di covarianza aggiornata
    % theta                - vettore dei parametri aggiornato
    % filtered_force       - forza filtrata attraverso IIR (N)
    % filtered_displacement- spostamento filtrato attraverso IIR (m)


    dt = p.dt;
    lambda = p.lambda;
    P = p.P;
    theta = p.theta;
    Kp_min = p.Kp_min;
    Kp_max = p.Kp_max;
    %% Parametri filtro IIR
    % Frequenza di taglio (cut-off frequency) = 10Hz
    f_cutoff = 10; 
    w_cutoff = 2*pi*f_cutoff; % pulsazione di taglio in rad/s
    
    % Definizione del filtro nel dominio continuo (Filtro passa-basso di primo ordine)
    num_continuous = [w_cutoff];
    den_continuous = [1 w_cutoff];
    
    % Trasformazione bilineare al dominio discreto (metodo Tustin)
    [num_discrete, den_discrete] = bilinear(num_continuous, den_continuous, 1/dt);
    
    % Inizializzazione stati filtro persistenti
    persistent force_state disp_state
    if isempty(force_state)
        force_state = zeros(max(length(den_discrete), length(num_discrete))-1,1);
        disp_state = zeros(max(length(den_discrete), length(num_discrete))-1,1);
    end
    
    %% Applicazione filtro digitale IIR
    [filtered_force, force_state] = filter(num_discrete, den_discrete, force_input, force_state);
    [filtered_displacement, disp_state] = filter(num_discrete, den_discrete, displacement_input, disp_state);
    
    %% Algoritmo RLS
    % Modello lineare da stimare:
    % filtered_force ≈ Kp * filtered_displacement
    
    phi = filtered_displacement; % regressore (input)
    y   = filtered_force;        % output (misura)
    
    % Guadagno di Kalman (aggiornamento RLS)
    epsilon = 1e-6; % small regularization term
    K_gain = (P * phi) / (lambda + phi' * P * phi + (eye(2) * epsilon));
    
    % Errore di predizione
    error_pred = y - phi' * theta;
    
    % Aggiornamento dei parametri (theta) stimati
    theta = theta + K_gain * error_pred;
    
    % Aggiornamento della matrice di covarianza (P)
    P = (P - K_gain * phi' * P) / lambda;
    
    % Valore stimato di rigidità Kp (evitando valori negativi)
    Kp_est = max(Kp_min, min(theta(1), Kp_max));
    
    end
    