%% holm_table.m  —  Formatea la tabla de comparaciones Holm step-down.
% ------------------------------------------------------------------
% Recibe el arreglo de structs 'pairwise' devuelto por friedman_analysis
% y devuelve una cadena de texto lista para imprimir.
% ------------------------------------------------------------------
function s = holm_table(pairs)
  s = '  Comparacion (vs control)   p_raw       p_Holm     Rechaza H0';
  for i = 1:numel(pairs)
    s = strjoin({s, sprintf('  %-22s  %.3e   %.3e     %s', ...
                              pairs(i).vs, pairs(i).p_raw, ...
                              pairs(i).p_holm, ...
                              tern(pairs(i).reject, 'SI', 'no'))}, '\n');
  end
end

function s = tern(cond, a, b)
  if cond, s = a; else, s = b; end
end
