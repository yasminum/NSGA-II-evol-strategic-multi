%% ode_opt.m  —  Online Differential Evolution (DE/rand/1/bin).
% ------------------------------------------------------------------
% Réplica fiel del Algorithm 1 de Rodríguez-Molina et al. (2024):
%   * mutación  V = X[r1] + F·(X[r2] − X[r3])      con F ~ U[0.3,0.9]
%     muestreado por generación (factor de escala aleatorizado)
%   * cruce binomial con CR = 0.5 y al menos un índice garantizado
%   * selección  (X[i], trials[i]) ← trials[i] si f(trials[i]) ≤ f(X[i])
%   * warm-start: el mejor individuo del proceso anterior (si existe)
%     se inyecta en X[1]
%   * presupuesto por llamada: NP × GMAX = 400 evaluaciones
%
% Sintaxis:
%   [x_best, f_best] = ode_opt(fbatch, dim, prev_best, pop, gens, CR)
% ------------------------------------------------------------------
function [x_best, f_best] = ode_opt(fbatch, dim, prev_best, pop, gens, CR)
  if nargin < 2 || isempty(dim),       dim = 9;       end
  if nargin < 3 || isempty(prev_best), prev_best = []; end
  if nargin < 4 || isempty(pop),       pop = 20;      end
  if nargin < 5 || isempty(gens),       gens = 20;     end
  if nargin < 6 || isempty(CR),        CR  = 0.5;     end

  X = rand(pop, dim);
  if ~isempty(prev_best)
    X(1,:) = prev_best(:).';
  end
  F = fbatch(X);

  for g = 1:(gens-1)
    Fw = 0.3 + 0.6 * rand();                % F por generación
    [r1, r2, r3] = distinct_triplets(pop);
    V = X(r1,:) + Fw * (X(r2,:) - X(r3,:));  % (pop,dim)
    CROSS = rand(pop, dim) < CR;
    % al menos un índice garantizado por fila
    jrand = randi(dim, pop, 1);
    for i = 1:pop
      CROSS(i, jrand(i)) = true;
    end
    trials = X;                              % begin from target
    trials(CROSS) = V(CROSS);
    trials = max(0, min(1, trials));         % clip al [0,1]
    Ft = fbatch(trials);
    imp = Ft <= F;
    X(imp,:) = trials(imp,:);
    F(imp)   = Ft(imp);
  end

  [f_best, b] = min(F);
  x_best = X(b,:);
end

function [r1, r2, r3] = distinct_triplets(pop)
% Índices (pop,3) con r1≠r2≠r3≠i por fila (muestreo uniforme equivalente).
  R = randi(pop, pop, 3);
  idx = (1:pop).';
  bad = any(R == idx, 2) | (R(:,1) == R(:,2)) | (R(:,1) == R(:,3)) ...
        | (R(:,2) == R(:,3));
  while any(bad)
    R(bad,:) = randi(pop, sum(bad), 3);
    bad = any(R == idx, 2) | (R(:,1) == R(:,2)) | (R(:,1) == R(:,3)) ...
          | (R(:,2) == R(:,3));
  end
  r1 = R(:,1); r2 = R(:,2); r3 = R(:,3);
end
