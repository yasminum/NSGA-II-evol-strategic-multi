%% get_optimizer.m  —  Despachador de los 4 algoritmos evolutivos online.
% ------------------------------------------------------------------
% Devuelve el function handle del algoritmo pedido, o lanza error si
% el nombre no existe. Se usa en AdaptiveCTC para instanciar el
% optimizador por nombre ("DE" | "GA" | "PSO" | "CMA").
% ------------------------------------------------------------------
function fn = get_optimizer(name)
  switch upper(name)
    case 'DE',  fn = @ode_opt;
    case 'GA',  fn = @oga_opt;
    case 'PSO', fn = @opso_opt;
    case 'CMA', fn = @ocma_opt;
    otherwise, error(['Optimizador desconocido: ' name]);
  end
end
