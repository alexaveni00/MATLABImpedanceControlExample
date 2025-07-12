function delta = HertzPenetration(m, k_HC)
% delta = HertzPenetration(...) calcola la penetrazione massima delta
% per il peso m_eff*g e restituisce la quota assoluta y_th = yinit - delta.
  g       = 9.81;
  % 3) forza statica
  F       = m * g;
  % 4) penetrazione massima
  delta   = (F / k_HC)^(2/3);
  % 5) soglia assoluta del terreno
end