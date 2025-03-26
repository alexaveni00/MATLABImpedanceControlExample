function [Kp_est, P, theta] = EstimateStiffness_IIR_RLS(force_input, displacement_input, p)
    % EstimateStiffness_IIR_RLS
    % Usa oggetti System di MATLAB per stimare la rigidità (Kp)
    % tramite filtro IIR e Recursive Least Squares (RLS)

    % Limiti di Kp
    Kp_min = p.Kp_min;
    Kp_max = p.Kp_max;

    % === Filtro IIR ===
    persistent iirForceFilter iirDispFilter
    if isempty(iirForceFilter)
        f_cutoff = 10; % Hz
        w_cutoff = 2*pi*f_cutoff;
        [b, a] = bilinear([w_cutoff], [1 w_cutoff], 1/p.dt);
        iirForceFilter = dsp.IIRFilter('Numerator', b, 'Denominator', a);
        iirDispFilter  = dsp.IIRFilter('Numerator', b, 'Denominator', a);
    end

    % Applica il filtro
    filtered_force = iirForceFilter(force_input);
    filtered_disp  = iirDispFilter(displacement_input);

    % === RLS nativo ===
    persistent rls
    if isempty(rls) || p.resetRLS
        rls = recursiveLS(2, 'ForgettingFactor', 0.98);
        rls.InitialParameters = p.theta; % Inizializza i parametri
        rls.InitialParameterCovariance = p.P;
        p.resetRLS = false;
    end

    % Costruisci vettore regressore (phi) 1x2
    phi = [filtered_disp, 1];   % primo elemento: regressore (disp), secondo: offset

    phi = reshape(phi, 1, []);

    % Applica stima RLS con l’oggetto System
    rls(filtered_force, phi);

    % Estrai parametri stimati
    theta = rls.Parameters; % [Kp; offset]
    P = rls.ParameterCovariance;

    % Stima finale Kp
    Kp_est = max(Kp_min, min(theta(1), Kp_max));
end
