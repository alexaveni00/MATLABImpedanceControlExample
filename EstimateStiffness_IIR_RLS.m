function [Kp_est, P, theta] = EstimateStiffness_IIR_RLS(force_input, displacement_input, p)
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
    theta = p.theta;  % attenzione: theta deve essere scalare!
    Kp_min = p.Kp_min;
    Kp_max = p.Kp_max;

    % Filtro IIR
    f_cutoff = 10; 
    w_cutoff = 2*pi*f_cutoff;
    
    num_continuous = [w_cutoff];
    den_continuous = [1 w_cutoff];
    
    [num_discrete, den_discrete] = bilinear(num_continuous, den_continuous, 1/dt);
    
    persistent force_state disp_state
    if isempty(force_state)
        force_state = zeros(max(length(den_discrete), length(num_discrete))-1,1);
        disp_state = zeros(max(length(den_discrete), length(num_discrete))-1,1);
    end
    
    [filtered_force, force_state] = filter(num_discrete, den_discrete, force_input, force_state);
    [filtered_displacement, disp_state] = filter(num_discrete, den_discrete, displacement_input, disp_state);

    % RLS (scalare)
    phi = [filtered_displacement; 1]; % vettore delle features
    y   = filtered_force;

    epsilon = 1e-6;
    K_gain = (P * phi) / (lambda + phi' * P * phi + epsilon);

    error_pred = y - phi' * theta;

    theta = theta + K_gain * error_pred;
    P = (P - K_gain * phi' * P) / lambda;

    % Stima finale Kp
    Kp_est = max(Kp_min, min(theta(1), Kp_max));
end