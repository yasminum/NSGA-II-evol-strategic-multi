%% monomials.m  —  Matriz de características del regresor polinomial.
% ------------------------------------------------------------------
% Devuelve la matriz de diseño Phi a partir de Z (N×9):
%
%   degree = 1 (regresor LINEAL, SLR):    Phi = [1, Z]
%                                            tamaño (N, 10)
%   degree = 2 (regresor POLINOMIAL, PR):  Phi = [1, Z, Z_i·Z_j  (i≤j)]
%                                            tamaño (N, 1 + 9 + 45 = 55)
%
% Los monomios de grado 2 se construyen con índices triu: para 9
% variables hay 9 cuadrados y 36 cruzados = 45 pares (i ≤ j).
%
% Como p = 55 > s_b = 10 muestras por ventana, el ajuste usa la
% solución de MÍNIMA NORMA (pinv), que es la generalización natural
% de la expresión cerrada (15) del paper cuando el sistema es
% indeterminado (más parámetros que observaciones).
% ------------------------------------------------------------------
function Phi = monomials(Z, degree)
  if nargin < 2, degree = 2; end
  N = size(Z, 1);
  ones_col = ones(N, 1);
  if degree <= 1
    Phi = [ones_col, Z];
    return;
  end
  [II, JJ] = find(triu(ones(9), 0));   % índices i ≤ j (pares, 9+36 = 45)
  Phi = [ones_col, Z, Z(:,II) .* Z(:,JJ)];
end
