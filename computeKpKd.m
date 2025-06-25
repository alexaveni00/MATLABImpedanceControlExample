function [Kp, Kd] = computeKpKd(thetadot)
    % computeKpKd - Calcola Kp (fisso) e Kd (adattivo) per il controllo ad impedenza.
    % Implementazione ispirata all'articolo di Semini et al. (2015),
    % "Towards versatile legged robots through active impedance control".
    %
    % - Kp è tenuto costante (come nei test riportati nell'articolo).
    % - Kd viene modulato dinamicamente in funzione della velocità verticale thetadot
    %   usando una funzione sigmoide come descritto nella sezione relativa alla Figura 9.
    %
    % INPUT:
    %   z  - quota verticale attuale del piede [m] (non usata in questa versione)
    %   thetadot - velocità verticale del piede [m/s]
    %
    % OUTPUT:
    %   Kp - guadagno proporzionale (costante) [N/m]
    %   Kd - guadagno derivativo adattivo [Ns/m]
    
    % Guadagno proporzionale fisso
    Kp = 150; 
    
    % --- Guadagno derivativo adattivo ---
    Kd0 = 6;   % valore base di smorzamento
    deltaKd = 9;   % valore massimo di smorzamento durante l'impatto
    Zs = 50;        % pendenza della sigmoide (più grande = transizione più rapida)
    
    % Funzione sigmoide in funzione della velocità angolare (thetadot)
    sigma = 1 / (1 + exp(-Zs * thetadot));
    
    % Interpolazione tra minimo e massimo smorzamento
    Kd = Kd0 + deltaKd * sigma;
    end
    