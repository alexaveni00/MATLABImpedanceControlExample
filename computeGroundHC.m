function [k_HC, c_HC] = computeGroundHC(E1,nu1,R1, E2,nu2,R2, e, m_eff)
% Calcola k and c per il modello Hunt–Crossley
% si suppone che l'end effector abbia una sfera alla fine
%   k  = (4/3)*E* * sqrt(R*)
%   c  = (3/2)*k * ((1-e^2)/(1+e)) * sqrt(m_eff/k)

    % 1) moduli ridotti ed equivalente
    E_star = 1 / ((1-nu1^2)/E1 + (1-nu2^2)/E2);

    R_star = 1 / (1/R1 + 1/R2); 
   

    % 2) k di Hunt–Crossley (esponente n=3/2)
    k_HC = (4/3) * E_star * sqrt(R_star); % (formula 2 Lankarani & Nikravesh 1990)

    % 3) coefficiente di damping
    % formula da Hunt&Crossley Sez.1.2 
    c_HC = (3/2) * k_HC * ((1 - e^2)/(1 + e)) * sqrt(m_eff / k_HC);
end
