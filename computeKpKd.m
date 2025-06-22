function [Kp, Kd] = computeKpKd(theta)
    % computeKpKd - Calcola Kp (fisso) e Kd (adattivo) per il controllo ad impedenza.
    % Implementazione ispirata all'articolo di Semini et al. (2015),
    % "Towards versatile legged robots through active impedance control".
    %
    % - Kp è tenuto costante (come nei test riportati nell'articolo).
    % - Kd viene modulato dinamicamente in funzione della velocità verticale theta
    %   usando una funzione sigmoide come descritto nella sezione relativa alla Figura 9.
    %
    % INPUT:
    %   z  - quota verticale attuale del piede [m] (non usata in questa versione)
    %   theta - velocità verticale del piede [m/s]
    %
    % OUTPUT:
    %   Kp - guadagno proporzionale (costante) [N/m]
    %   Kd - guadagno derivativo adattivo [Ns/m]
    
    % Guadagno proporzionale variabile (non in Semini, ma coerente con la logica)
    Kp = 150; 
    
    % --- Guadagno derivativo adattivo ---
    Kd0 = 6;   % valore base di smorzamento
    deltaKd = 9;   % valore massimo di smorzamento durante l'impatto
    Zs = 10;        % pendenza della sigmoide (più grande = transizione più rapida)
    
    % Funzione sigmoide in funzione della velocità angolare (theta)
    sigma = 1 / (1 + exp(-Zs * theta));
    
    % Interpolazione tra minimo e massimo smorzamento
    Kd = Kd0 + deltaKd * sigma;
    end
    